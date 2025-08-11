#include "LIS2MDL.h"
#include "LSM6DSL.h"

#include <Wire.h>

// Create sensor objects
LSM6DSL imu;
LIS2MDL mag;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

void setup() {
  Serial.begin(921600);

  delay(200);
  // Serial.println("=== Sensor Demo: LSM6DSL + LIS2MDL ===");

  // Initialize I2C (default SDA=21, SCL=22 on ESP32)
  Wire.begin(8, 10);

  // Initialize LSM6DSL (accelerometer + gyroscope)
  if (!imu.beginI2C(Wire)) {
    // Serial.println("Failed to initialize LSM6DSL!");
    while (1) delay(1000);
  }

  // Initialize LIS2MDL (magnetometer)
  if (!mag.beginI2C(Wire)) {
    // Serial.println("Failed to initialize LIS2MDL!");
    while (1) delay(1000);
  }
}

void loop() {
  // dt = esp_timer_get_time() - now_ns;
  if (Serial.read() == (int)'j') {

    // 1) Capture timestamp *right before/after* reading the sensor

    // 2) Read the sensors (fast path)
    imu.readData(ax, ay, az, gx, gy, gz, true);
    mag.readData(mx, my, mz, true);

    // 3) Print CSV: t_us,idx,ax,ay,az,gx,gy,gz,mx,my,mz
    Serial.printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                  ax, ay, az, gx, gy, gz, mx, my, mz);
  }
}
