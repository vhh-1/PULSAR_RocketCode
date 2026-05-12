## PULSAR ROCKET CODE
### Arduino
Takes raw data from gyro+accelerometer (6 bytes), time,  and sends them @~200Hz

### Python
Recieves and plots Z accel live

## ToDo
### Arduino
- Get Data from Barometric and Temp
- get time from polling
- send that data
- save data to SD card

### Python
- Improve derivative tracking(RK4???)
- Use Gyro data to normalize Z
- plot gyro vs baro altitude
- some sort of stability checking using the regularity of time intervals.
  
