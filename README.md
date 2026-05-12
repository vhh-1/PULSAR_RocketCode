## PULSAR ROCKET CODE
### Arduino
Takes raw data from gyro+accelerometer (6*4 bytes),  and sends them @~200Hz

### Python
Recieves and plots Z accel live

## ToDo
### Arduino
- Get Data from Barometric and Temp
- get time actual data time
- send that data
- save data to SD card

### Python
#### Data Gathering
- use Real dts
- some sort of stability checking using the regularity of time intervals.
#### Data Processing
- Improve integral tracking(RK4???)
- Use Gyro data to normalize Z
- plot gyro vs baro altitude

  
