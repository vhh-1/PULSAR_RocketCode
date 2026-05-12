#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>

// --- Network Settings ---
char ssid[] = "Hootspoot";      
char pass[] = "Veenerschnitzel!";  
IPAddress pcIP(10,102,154,172); // Replace with your computer's Static IP
unsigned int pcPort = 4210;      

#define BATCH_SIZE 10

struct SensorData {
  float ax, ay, az; // Accelerometer
  float gx, gy, gz; // Gyroscope
};

SensorData buffer[BATCH_SIZE];
int bufferIndex = 0;
WiFiUDP Udp;

void setup() {
  if (!IMU.begin()) while (1);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) delay(500);
  Udp.begin(2390);
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(buffer[bufferIndex].ax, buffer[bufferIndex].ay, buffer[bufferIndex].az);
    IMU.readGyroscope(buffer[bufferIndex].gx, buffer[bufferIndex].gy, buffer[bufferIndex].gz);
    bufferIndex++;

    if (bufferIndex >= BATCH_SIZE) {
      Udp.beginPacket(pcIP, pcPort);
      Udp.write((uint8_t*)buffer, sizeof(buffer)); // 240 bytes
      Udp.endPacket();
      bufferIndex = 0;
    }
  }
}