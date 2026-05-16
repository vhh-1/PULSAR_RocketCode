import socket
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIG ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
BATCH_SIZE = 10
# 7 elements per sample (6 floats + 1 uint32) = 28 bytes * 10 samples = 280 bytes
EXPECTED_SIZE = 280 
packet_format = "<" + "ffffffI" * BATCH_SIZE 

# --- DATA STORAGE ---
MAX_POINTS = 400
accel_z_data = [0.0] * MAX_POINTS
vel_z_data = [0.0] * MAX_POINTS
dist_z_data = [0.0] * MAX_POINTS

# State
v_z, d_z = 0.0, 0.0
offset_z = 1.0 
last_dev_time = None
is_calibrating = False
calib_buffer = []

# Open File for logging (Gyro + Accel)
log_file = open("rocket_sensor_log2.csv", "w")
log_file.write("pc_timestamp,ax,ay,az,gx,gy,gz,dev_time\n")

# Init Wifi connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Init Plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
plt.subplots_adjust(hspace=0.5)

# Calibrate Func
def on_press(event):
    global is_calibrating, calib_buffer, v_z, d_z
    if event.key == 'c':
        print("Calibrating... Keep the Nano still!")
        is_calibrating = True
        calib_buffer = []
        v_z, d_z = 0.0, 0.0
fig.canvas.mpl_connect('key_press_event', on_press)

# Updating graphs
def update(frame):
    global v_z, d_z, last_dev_time, offset_z, is_calibrating, calib_buffer
    
    try:
        while True:
            try:
                # Get Data
                data, addr = sock.recvfrom(1024)
                num_batches = len(data) // EXPECTED_SIZE
                pc_now = time.time()
                
                # Unpack and begin data processing
                for b in range(num_batches):
                    chunk = data[b * EXPECTED_SIZE : (b + 1) * EXPECTED_SIZE]
                    values = struct.unpack(packet_format, chunk)
                    
                    # Prepare a batch of lines to write to CSV efficiently
                    csv_lines = []
                    
                    # FIXED: Step by 7 because your format has 7 elements per sample
                    for i in range(0, len(values), 7):
                        ax, ay, az, gx, gy, gz, dev_time = values[i:i+7]
                        
                        # 1. Accumulate raw data for a quick batch log write
                        csv_lines.append(f"{pc_now},{ax},{ay},{az},{gx},{gy},{gz},{dev_time}\n")

                        # Calibration Code
                        if is_calibrating:
                            calib_buffer.append(az)
                            if len(calib_buffer) >= 150: # Average 150 samples
                                offset_z = sum(calib_buffer) / len(calib_buffer)
                                is_calibrating = False
                                print(f"Calibrated! New Offset: {offset_z:.4f}")
                            continue

                        # 2. Timing calculation using microcontroller clock (assumes millis)
                        if last_dev_time is None:
                            sample_dt = 0.01  # default fallback for first loop
                        else:
                            # Convert delta-millis to seconds
                            sample_dt = (dev_time - last_dev_time) / 1000.0
                            # Handle timer rollover just in case
                            if sample_dt < 0 or sample_dt > 1.0: 
                                sample_dt = 0.01
                        
                        last_dev_time = dev_time

                        # 3. Acceleration Logic (Gravity removal)
                        a_raw = (az - offset_z) * 9.81
                        
                        # 4. Integrate with 2% Damping to stop drift
                        v_z = (v_z + a_raw * sample_dt) * 0.98 
                        
                        # Noise floor moved to velocity as per your code comment
                        if abs(v_z) < 0.05: 
                            v_z = 0.0 
                            
                        d_z += v_z * sample_dt
                        
                        # Store for plotting
                        accel_z_data.append(a_raw); accel_z_data.pop(0)
                        vel_z_data.append(v_z); vel_z_data.pop(0)
                        dist_z_data.append(d_z); dist_z_data.pop(0)
                    
                    # Bulk write this packet's data to disk outside the unpacking loop
                    log_file.writelines(csv_lines)

            except BlockingIOError:
                break 
    except Exception as e:
        print(f"Error processing packet: {e}")

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
    
    # Don't let Y-axis zoom closer than 10cm total range
    c_min, c_max = min(dist_z_data), max(dist_z_data)
    if (c_max - c_min) < 0.10:
        mid = (c_max + c_min) / 2
        ax3.set_ylim(mid - 0.05, mid + 0.05)

print("Starting receiver... Click the graph window and press 'c' to zero out gravity.")
ani = FuncAnimation(fig, update, interval=20)
plt.show()

# Clean up
log_file.close()