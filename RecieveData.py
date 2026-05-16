import socket
import struct
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIG ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
BATCH_SIZE = 10
# 10 samples * (7 floats * 4 bytes) = 280 bytes expected per UDP packet
EXPECTED_SIZE = 280  
packet_format = "<" + "fffffff" * BATCH_SIZE

# --- DATA STORAGE ---
MAX_POINTS = 400
accel_z_data = [0.0] * MAX_POINTS
vel_z_data = [0.0] * MAX_POINTS
dist_z_data = [0.0] * MAX_POINTS

# --- STATE VARIABLES ---
v_z, d_z = 0.0, 0.0

# 3D Orientation Quaternion
q = [1.0, 0.0, 0.0, 0.0] 

# Calibration Offsets (Starts as True to auto-calibrate on boot)
is_calibrating = True
calib_buffer = []  
gyro_bias = [0.0, 0.0, 0.0]
accel_scale = 1.0  
last_a_raw = 0.0 

print("System initialized. Auto-calibrating immediately upon receiving data...")
print("Keep the device completely flat and still!")

# Open File for logging
log_file = open("rocket_sensor_log.csv", "w")
log_file.write("pc_timestamp,dt,ax,ay,az,gx,gy,gz,true_vertical_accel,v_z,d_z\n")

# Init Wifi connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Init Plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
plt.subplots_adjust(hspace=0.5)

# Manual Calibration Trigger (Press 'c' on the graph window to re-calibrate)
def on_press(event):
    global is_calibrating, calib_buffer, v_z, d_z, q, last_a_raw
    if event.key == 'c':
        print("\nRe-calibrating... Keep the device perfectly flat and still!")
        is_calibrating = True
        calib_buffer = []
        v_z, d_z = 0.0, 0.0
        last_a_raw = 0.0
        q = [1.0, 0.0, 0.0, 0.0]  

fig.canvas.mpl_connect('key_press_event', on_press)

def update(frame):
    global v_z, d_z, is_calibrating, calib_buffer, gyro_bias, accel_scale, q, last_a_raw
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                num_batches = len(data) // EXPECTED_SIZE
                csv_lines = []

                for b in range(num_batches):
                    chunk = data[b * EXPECTED_SIZE : (b + 1) * EXPECTED_SIZE]
                    values = struct.unpack(packet_format, chunk)
                    
                    for i in range(0, len(values), 7):
                        # The 7th element is now the pure sample_dt float directly from the Arduino
                        ax, ay, az, gx, gy, gz, sample_dt = values[i:i+7]
                        
                        # Fallback sanity check against packet drops or network hitching
                        if sample_dt <= 0 or sample_dt > 0.5:
                            sample_dt = 0.01  

                        # --- CALIBRATION PHASE ---
                        if is_calibrating:
                            calib_buffer.append([ax, ay, az, gx, gy, gz])
                            if len(calib_buffer) >= 150:
                                gyro_bias[0] = sum(s[3] for s in calib_buffer) / len(calib_buffer)
                                gyro_bias[1] = sum(s[4] for s in calib_buffer) / len(calib_buffer)
                                gyro_bias[2] = sum(s[5] for s in calib_buffer) / len(calib_buffer)
                                
                                avg_ax = sum(s[0] for s in calib_buffer) / len(calib_buffer)
                                avg_ay = sum(s[1] for s in calib_buffer) / len(calib_buffer)
                                avg_az = sum(s[2] for s in calib_buffer) / len(calib_buffer)
                                total_g = math.sqrt(avg_ax**2 + avg_ay**2 + avg_az**2)
                                if total_g > 0:
                                    accel_scale = 1.0 / total_g
                                    
                                is_calibrating = False
                                print(f"Calibrated successfully! Gyro Biases: {gyro_bias}, Scale: {accel_scale:.4f}")
                            continue

                        # --- 1. REMOVE BIASES & CONVERT TO RADIANS ---
                        gx_clean = gx - gyro_bias[0]
                        gy_clean = gy - gyro_bias[1]
                        gz_clean = gz - gyro_bias[2]

                        # CRITICAL: Quaternion math requires radians/second. 
                        # If your Arduino sends radians natively, remove the math.radians() wrap.
                        gx_rad = math.radians(gx_clean)
                        gy_rad = math.radians(gy_clean)
                        gz_rad = math.radians(gz_clean)

                        # --- 2. UPDATE QUATERNION ---
                        q0, q1, q2, q3 = q
                        half_dt = sample_dt * 0.5
                        
                        # Integrate orientation using RADIANS
                        q0_new = q0 + half_dt * (-q1*gx_rad - q2*gy_rad - q3*gz_rad)
                        q1_new = q1 + half_dt * ( q0*gx_rad - q3*gy_rad + q2*gz_rad)
                        q2_new = q2 + half_dt * ( q3*gx_rad + q0*gy_rad - q1*gz_rad)
                        q3_new = q3 + half_dt * (-q2*gx_rad + q1*gy_rad + q0*gz_rad)
                        
                        norm = math.sqrt(q0_new**2 + q1_new**2 + q2_new**2 + q3_new**2)
                        if norm > 0:
                            q = [q0_new/norm, q1_new/norm, q2_new/norm, q3_new/norm]
                        q0, q1, q2, q3 = q

                        # --- 3. EARTH-FRAME ROTATION (FIXED MATRIX) ---
                        ax_s = ax * accel_scale
                        ay_s = ay * accel_scale
                        az_s = az * accel_scale

                        # This is the correct Local-to-Global Z-axis projection matrix formula
                        global_a_z = 2.0 * (q1*q3 - q0*q2) * ax_s + 2.0 * (q2*q3 + q0*q1) * ay_s + (q0**2 - q1**2 - q2**2 + q3**2) * az_s
                        
                        a_raw = (global_a_z * 9.80665) - 9.80665

                        # --- 4. TRAPEZOIDAL INTEGRATION ---
                        # Predict next velocity step
                        v_z_next = v_z + 0.5 * (last_a_raw + a_raw) * sample_dt
                        
                        # Integrate distance USING velocity BEFORE deadband cutoff
                        d_z += 0.5 * (v_z + v_z_next) * sample_dt
                        
                        # Finalize velocity state for the next loop
                        v_z = v_z_next
                        last_a_raw = a_raw

                        # Deadband noise filter to stop resting drift
                        if abs(a_raw) < 0.15 and abs(v_z) < 0.10:
                            v_z = 0.0

                        # Append to CSV logging buffer
                        csv_lines.append(f"{time.time()},{sample_dt},{ax},{ay},{az},{gx},{gy},{gz},{a_raw},{v_z},{d_z}\n")

                        # Manage graph plotting arrays
                        accel_z_data.append(a_raw); accel_z_data.pop(0)
                        vel_z_data.append(v_z); vel_z_data.pop(0)
                        dist_z_data.append(d_z); dist_z_data.pop(0)

                # Write log chunks to disk
                if csv_lines:
                    log_file.writelines(csv_lines)

            except BlockingIOError:
                break
    except Exception as e:
        pass

    # UI Visual Refresh Updates
    ax1.clear()
    ax1.plot(accel_z_data, 'r')
    ax1.set_ylabel('True Vertical Accel (m/s²)')
    ax1.set_title(f"Vel: {v_z:.2f} m/s | Alt: {d_z:.2f} m | Press 'c' to Re-Calibrate")
    
    ax2.clear()
    ax2.plot(vel_z_data, 'g')
    ax2.set_ylabel('Vertical Velocity (m/s)')
    
    ax3.clear()
    ax3.plot(dist_z_data, 'b')
    ax3.set_ylabel('True Altitude (m)')

    c_min, c_max = min(dist_z_data), max(dist_z_data)
    if (c_max - c_min) < 0.10:
        mid = (c_max + c_min) / 2
        ax3.set_ylim(mid - 0.05, mid + 0.05)

print("Starting graphical UI stream receiver...")
ani = FuncAnimation(fig, update, interval=20)
plt.show()

log_file.close()