#include "LIS2MDL.h"
#include "LSM6DSL.h"

#include <Wire.h>

// Create sensor objects
LSM6DSL imu;
LIS2MDL mag;

void setup() {
  Serial.begin(115200);

  delay(200);

  Serial.println("=== Sensor Demo: LSM6DSL + LIS2MDL ===");

  // Initialize I2C (default SDA=21, SCL=22 on ESP32)
  Wire.begin(   );

  // Initialize LSM6DSL (accelerometer + gyroscope)
  if (!imu.beginI2C(Wire)) {
    Serial.println("Failed to initialize LSM6DSL!");
    while (1) delay(1000);
  }

  // Initialize LIS2MDL (magnetometer)
  if (!mag.beginI2C(Wire)) {
    Serial.println("Failed to initialize LIS2MDL!");
    while (1) delay(1000);
  }

  Serial.println("Initialization successful.");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  // Read raw data
  imu.readData(ax, ay, az,
               gx, gy, gz);
  mag.readData(mx, my, mz);

  // // Print accelerometer data
  Serial.print("Accel [m/s^2]  : ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);

  // // Print gyroscope data
  Serial.print("Gyro  [deg/s]  : ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  // Print magnetometer data
  Serial.print("Mag   [uT]  : ");
  Serial.print(mx*1000000); Serial.print(", ");
  Serial.print(my*1000000); Serial.print(", ");
  Serial.println(mz*1000000);

  Serial.println();
  delay(200);
}