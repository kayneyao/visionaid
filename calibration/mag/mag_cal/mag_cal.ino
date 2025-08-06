#include "LIS2MDL.h"
#include "LSM6DSL.h"

#include <Wire.h>

// Create sensor objects
LSM6DSL imu;
LIS2MDL mag;

const float hard_iron[3] = {
  2.82, -1.33, -2.60
};

const float soft_iron[3][3] = {
  {0.981, 0.017, -0.012},
  {0.017, 1.015, 0.014},
  {-0.012, 0.014, 1.006}
};

const float mag_field = 42.88;

void setup() {
  Serial.begin(115200);

  delay(200);

  // Serial.println("=== Sensor Demo: LSM6DSL + LIS2MDL ===");

  // Initialize I2C (default SDA=21, SCL=22 on ESP32)
  Wire.begin(8, 10);

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

  // Serial.println("Initialization successful.");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  float hcal_mx, hcal_my, hcal_mz;

  float mag_data[3];

  // Read raw data
  imu.readData(ax, ay, az,
               gx, gy, gz);
  mag.readData(mx, my, mz);

  hcal_mx = mx - hard_iron[0];
  hcal_my = my - hard_iron[1];
  hcal_mz = mz - hard_iron[2];

  for(int i = 0; i < 3; i++){
    mag_data[i] = (soft_iron[i][0] * hcal_mx) +
                  (soft_iron[i][1] * hcal_my) +
                  (soft_iron[i][2] * hcal_mz);
  }

  // Print accelerometer data
  Serial.print("Raw:");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");

  // // Print gyroscope data
  // Serial.print("Gyro  [deg/s]  : ");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");

  // Print magnetometer data
  // Serial.print("Raw:");
  // Serial.print(int(mx*10)); Serial.print(",");
  // Serial.print(int(my*10)); Serial.print(",");
  // Serial.println(int(mz*10));

  // Serial.print("Cal:");
  Serial.print(int(mag_data[0]*10)); Serial.print(",");
  Serial.print(int(mag_data[1]*10)); Serial.print(",");
  Serial.println(int(mag_data[2]*10));

  // Serial.print("Uni:");
  // Serial.print(0); Serial.print(",");
  // Serial.print(0); Serial.print(",");
  // Serial.print(0); Serial.print(",");

  // // // Print gyroscope data
  // // Serial.print("Gyro  [deg/s]  : ");
  // Serial.print(0); Serial.print(",");
  // Serial.print(0); Serial.print(",");
  // Serial.print(0); Serial.print(",");

  // // Print magnetometer data
  // // Serial.print("Raw:");
  // Serial.print(mx); Serial.print(",");
  // Serial.print(my); Serial.print(",");
  // Serial.println(mz);

  

  delay(10);
}