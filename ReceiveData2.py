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
EXPECTED_SIZE = 280  # 10 samples * 6 floats * 4 bytes
packet_format = "<" + "ffffff" * BATCH_SIZE

# --- DATA STORAGE ---
MAX_POINTS = 400
accel_z_data = [0.0] * MAX_POINTS
vel_z_data = [0.0] * MAX_POINTS
dist_z_data = [0.0] * MAX_POINTS

# --- STATE VARIABLES ---
v_z, d_z = 0.0, 0.0
last_packet_time = time.time()

# 3D Orientation Quaternion (Initializes to pointing straight up)
# Coordinates assume local sensor configuration aligns with gravity at boot.
q = [1.0, 0.0, 0.0, 0.0] 

# Calibration Offsets
is_calibrating = False
calib_buffer = []  # Stores arrays of [ax, ay, az, gx, gy, gz]
gyro_bias = [0.0, 0.0, 0.0]
accel_scale = 1.0  # Scalar modifier to match 1.0G target

# Open File for logging (Gyro + Accel)
log_file = open("rocket_sensor_log2.csv", "w")
log_file.write("timestamp,ax,ay,az,gx,gy,gz,global_az,v_z,d_z\n")

# Init Wifi connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Init Plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
plt.subplots_adjust(hspace=0.5)

# Calibration Trigger
def on_press(event):
    global is_calibrating, calib_buffer, v_z, d_z, q
    if event.key == 'c':
        print("Calibrating... Keep the device perfectly flat and still!")
        is_calibrating = True
        calib_buffer = []
        v_z, d_z = 0.0, 0.0
        q = [1.0, 0.0, 0.0, 0.0]  # Reset orientation tracking

fig.canvas.mpl_connect('key_press_event', on_press)

# Pure physics RK4 derivative function helper
def derivatives(v, a_true):
    return v, a_true

# Updating graphs and parsing incoming data packets
def update(frame):
    global v_z, d_z, last_packet_time, is_calibrating, calib_buffer, gyro_bias, accel_scale, q
    
    try:
        while True:
            try:
                # Get Data
                data, addr = sock.recvfrom(1024)
                num_batches = len(data) // EXPECTED_SIZE
                
                # Calculate timing loops
                current_time = time.time()
                packet_dt = current_time - last_packet_time
                last_packet_time = current_time
                sample_dt = packet_dt / (num_batches * BATCH_SIZE)
                
                # Protect against sudden zero-divisions
                if sample_dt <= 0:
                    sample_dt = 0.01

                # Log whole chunks outside individual iterations to lower disk overhead
                csv_lines = []

                for b in range(num_batches):
                    chunk = data[b * EXPECTED_SIZE : (b + 1) * EXPECTED_SIZE]
                    values = struct.unpack(packet_format, chunk)
                    
                    for i in range(0, len(values), 6):
                        ax, ay, az, gx, gy, gz = values[i:i+6]
                        
                        # Calibration Code Profiling
                        if is_calibrating:
                            calib_buffer.append([ax, ay, az, gx, gy, gz])
                            if len(calib_buffer) >= 150:
                                # Average gyroscope offsets
                                gyro_bias[0] = sum(s[3] for s in calib_buffer) / len(calib_buffer)
                                gyro_bias[1] = sum(s[4] for s in calib_buffer) / len(calib_buffer)
                                gyro_bias[2] = sum(s[5] for s in calib_buffer) / len(calib_buffer)
                                
                                # Scale acceleration reading so rest state magnitude equals exactly 1.0
                                avg_ax = sum(s[0] for s in calib_buffer) / len(calib_buffer)
                                avg_ay = sum(s[1] for s in calib_buffer) / len(calib_buffer)
                                avg_az = sum(s[2] for s in calib_buffer) / len(calib_buffer)
                                total_g = math.sqrt(avg_ax**2 + avg_ay**2 + avg_az**2)
                                if total_g > 0:
                                    accel_scale = 1.0 / total_g
                                    
                                is_calibrating = False
                                print(f"Calibrated! Gyro Biases: {gyro_bias}, Scale Factor: {accel_scale:.4f}")
                            continue

                        # --- STEP 1: REMOVE SENSOR GYRO BIASES ---
                        gx_clean = gx - gyro_bias[0]
                        gy_clean = gy - gyro_bias[1]
                        gz_clean = gz - gyro_bias[2]

                        # --- STEP 2: UPDATE ORIENTATION QUATERNION (RK1 integration for speed) ---
                        # Gyro values assumed to be in radians/sec.
                        q0, q1, q2, q3 = q
                        half_dt = sample_dt * 0.5
                        
                        q0_new = q0 + half_dt * (-q1*gx_clean - q2*gy_clean - q3*gz_clean)
                        q1_new = q1 + half_dt * ( q0*gx_clean - q3*gy_clean + q2*gz_clean)
                        q2_new = q2 + half_dt * ( q3*gx_clean + q0*gy_clean - q1*gz_clean)
                        q3_new = q3 + half_dt * (-q2*gx_clean + q1*gy_clean + q0*gz_clean)
                        
                        # Normalize quaternion to prevent numerical degradation over time
                        norm = math.sqrt(q0_new**2 + q1_new**2 + q2_new**2 + q3_new**2)
                        if norm > 0:
                            q = [q0_new/norm, q1_new/norm, q2_new/norm, q3_new/norm]
                        q0, q1, q2, q3 = q

                        # --- STEP 3: ROTATE ACCELERATION VECTOR TO THE GLOBAL FRAME ---
                        # Scale raw input to reference units
                        ax_s = ax * accel_scale
                        ay_s = ay * accel_scale
                        az_s = az * accel_scale

                        # Extract the Z-row of the Quaternion rotation matrix to isolate True vertical Gs
                        # This equates to finding the dot product of global vertical against orientation states.
                        global_a_z = 2.0 * (q1*q3 - q0*q2) * ax_s + 2.0 * (q0*q1 + q2*q3) * ay_s + (q0**2 - q1**2 - q2**2 + q3**2) * az_s
                        
                        # Convert localized G units into meters per second squared, then extract gravity
                        a_raw = (global_a_z * 9.81) - 9.81

                        # --- STEP 4: INTEGRATE VIA RUNGE-KUTTA 4 ---
                        k1_v, k1_a = derivatives(v_z, a_raw)
                        k2_v, k2_a = derivatives(v_z + 0.5 * sample_dt * k1_a, a_raw)
                        k3_v, k3_a = derivatives(v_z + 0.5 * sample_dt * k2_a, a_raw)
                        k4_v, k4_a = derivatives(v_z + sample_dt * k3_a, a_raw)

                        # Update global states
                        d_z += (sample_dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
                        v_z += (sample_dt / 6.0) * (k1_a + 2.0 * k2_a + 2.0 * k3_a + k4_a)

                        # Apply floor dynamics to velocity state
                        v_z *= 0.98
                        if abs(v_z) < 0.1:
                            v_z = 0.0

                        # Append status to log string array
                        csv_lines.append(f"{time.time()},{ax},{ay},{az},{gx},{gy},{gz},{a_raw},{v_z},{d_z}\n")

                        # Store data values for plotting
                        accel_z_data.append(a_raw); accel_z_data.pop(0)
                        vel_z_data.append(v_z); vel_z_data.pop(0)
                        dist_z_data.append(d_z); dist_z_data.pop(0)

                # Dump calculated tracking data down to the log file in a single IO step
                if csv_lines:
                    log_file.writelines(csv_lines)

            except BlockingIOError:
                break
    except Exception as e:
        pass

    # Plotting Data Visualization Refresh
    ax1.clear()
    ax1.plot(accel_z_data, 'r')
    ax1.set_ylabel('True Vertical Accel (m/s²)')
    ax1.set_title(f"Vel: {v_z:.2f} m/s | Alt: {d_z:.2f} m | Press 'c' to Calibrate")
    
    ax2.clear()
    ax2.plot(vel_z_data, 'g')
    ax2.set_ylabel('Vertical Velocity (m/s)')
    
    ax3.clear()
    ax3.plot(dist_z_data, 'b')
    ax3.set_ylabel('True Altitude (m)')

    # Maintain scannable visual dynamic windows minimum 10cm bounds
    c_min, c_max = min(dist_z_data), max(dist_z_data)
    if (c_max - c_min) < 0.10:
        mid = (c_max + c_min) / 2
        ax3.set_ylim(mid - 0.05, mid + 0.05)

print("Starting receiver... Click the graph window and press 'c' to calibrate orientation.")
ani = FuncAnimation(fig, update, interval=20)
plt.show()

# Clean up resource files safely upon termination
log_file.close()
