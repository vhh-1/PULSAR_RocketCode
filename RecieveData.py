import socket
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIG ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
BATCH_SIZE = 10
EXPECTED_SIZE = 280 # 10 samples * 6 floats (3 accel + 3 gyro) * 4 bytes
packet_format = "<" + "ffffff" * BATCH_SIZE 

# --- DATA STORAGE ---
MAX_POINTS = 400
accel_z_data = [0.0] * MAX_POINTS
vel_z_data = [0.0] * MAX_POINTS
dist_z_data = [0.0] * MAX_POINTS

# State
v_z, d_z = 0.0, 0.0
offset_z = 1.0 
last_packet_time = time.time()
is_calibrating = False
calib_buffer = []

# Open File for logging (Gyro + Accel)
log_file = open("rocket_sensor_log2.csv", "w")
log_file.write("timestamp,ax,ay,az,gx,gy,gz,time\n")

#init Wifi connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

#init Plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
plt.subplots_adjust(hspace=0.5)

#Calibrate Func
def on_press(event):
    global is_calibrating, calib_buffer, v_z, d_z
    if event.key == 'c':
        print("Calibrating... Keep the Nano still!")
        is_calibrating = True
        calib_buffer = []
        v_z, d_z = 0.0, 0.0
fig.canvas.mpl_connect('key_press_event', on_press)

#Updating graphs
def update(frame):
    global v_z, d_z, last_packet_time, offset_z, is_calibrating, calib_buffer
    try:
        while True:
            try:
                #Get Data
                data, addr = sock.recvfrom(1024)
                num_batches = len(data) // EXPECTED_SIZE
                
                # Calculate timing (Update this to use new dt code)
                current_time = time.time()
                packet_dt = current_time - last_packet_time
                last_packet_time = current_time
                sample_dt = packet_dt / (num_batches * BATCH_SIZE)
                
                #unpack and being datga processing
                for b in range(num_batches):
                    chunk = data[b * EXPECTED_SIZE : (b + 1) * EXPECTED_SIZE]
                    values = struct.unpack(packet_format, chunk)
                    
                    for i in range(0, len(values), 6):
                        ax, ay, az, gx, gy, gz = values[i:i+6]
                        
                        # 1. Log all raw data to CSV (including Gyro)(THIS IS THE IMPORTANT PART. THE REST IS LIVE DATA PLOTTING)
                        # We should have it do this outside the loop, just a dump of all the data it recieves
                        log_file.write(f"{time.time()},{ax},{ay},{az},{gx},{gy},{gz}\n")

                        # Calibration Code
                        if is_calibrating:
                            calib_buffer.append(az)
                            if len(calib_buffer) >= 150: # Average 150 samples
                                offset_z = sum(calib_buffer) / len(calib_buffer)
                                is_calibrating = False
                                print(f"Calibrated! New Offset: {offset_z:.4f}")
                            continue

                        # 2. Acceleration Logic(graviry and muting)
                        a_raw = (az - offset_z) * 9.81
                        if abs(a_raw) < 0.25: a_raw = 0 # Noise floor(maybe get rid of this as it could cause issue and put the mute on the vel instead.)
                        
                        # 3. Integrate with 2% Damping to stop drift
                        #RK4 from 210????
                        v_z = (v_z + a_raw * sample_dt) * 0.98 
                        if abs(v_z) < 0.1: v_z = 0 # Noise floor
                        d_z += v_z * sample_dt
                        
                        # Store for plotting
                        accel_z_data.append(a_raw); accel_z_data.pop(0)
                        vel_z_data.append(v_z); vel_z_data.pop(0)
                        dist_z_data.append(d_z); dist_z_data.pop(0)

            except BlockingIOError:
                break 
    except Exception as e:
        pass

    # Plotting
    ax1.clear()
    ax1.plot(accel_z_data, 'r')
    ax1.set_ylabel('Accel Z (m/s²)')
    ax1.set_title(f"Z-Offset: {offset_z:.4f} | Vel: {v_z:.2f} m/s | Press 'c' to Calibrate")
    
    ax2.clear()
    ax2.plot(vel_z_data, 'g')
    ax2.set_ylabel('Velocity Z (m/s)')
    
    ax3.clear()
    ax3.plot(dist_z_data, 'b')
    ax3.set_ylabel('Distance (m)')
    
    #Don't let Y-axis zoom closer than 10cm total range
    c_min, c_max = min(dist_z_data), max(dist_z_data)
    if (c_max - c_min) < 0.10:
        mid = (c_max + c_min) / 2
        ax3.set_ylim(mid - 0.05, mid + 0.05)

print("Starting receiver... Click the graph window and press 'c' to zero out gravity.")
ani = FuncAnimation(fig, update, interval=20)
plt.show()

# Clean up
log_file.close()