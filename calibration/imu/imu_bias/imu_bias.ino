#include <Wire.h>

// I2C addresses
#define LSM6DSL_ADDR  0x6A
#define LIS2MDL_ADDR  0x1E

// LSM6DSL regs
#define CTRL1_XL   0x10
#define CTRL2_G    0x11
#define CTRL5_C    0x14
#define CTRL8_XL   0x17
#define OUTX_L_XL  0x28
#define OUTX_L_G   0x22

// LIS2MDL regs
#define CFG_REG_A  0x60
#define CFG_REG_B  0x61
#define CFG_REG_C  0x62
#define OUTX_L_M   0x68

// How long to log in RAW mode (milliseconds)
const unsigned long LOG_DURATION_MS = 14400000UL; // e.g. 4 h

// Hard-iron offsets (µT)
const float hard_iron[3] = {
  2.82f, -1.33f, -2.60f
};
// Soft-iron correction matrix
const float soft_iron[3][3] = {
  {0.981f, 0.017f, -0.012f},
  {0.017f, 1.015f,  0.014f},
  {-0.012f,0.014f,  1.006f}
};

// helper: write one byte
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// helper: read signed 16-bit little-endian
int16_t i2cRead16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  uint8_t lo = Wire.read(), hi = Wire.read();
  return (int16_t)(hi << 8 | lo);
}

// RAW IMU setup: 104 Hz ±2 g/±245 dps, filters OFF
void initIMURaw() {
  i2cWrite(LSM6DSL_ADDR, CTRL1_XL, 0x40);
  i2cWrite(LSM6DSL_ADDR, CTRL2_G,  0x40);
  i2cWrite(LSM6DSL_ADDR, CTRL8_XL, 0x00);
  i2cWrite(LSM6DSL_ADDR, CTRL5_C,  0x00);
}

// RAW mag setup: 100 Hz, temp-comp ON, no LPF/offset, no BDU
void initMagRaw() {
  i2cWrite(LIS2MDL_ADDR, CFG_REG_A, 0x80 | 0x0C);
  i2cWrite(LIS2MDL_ADDR, CFG_REG_B, 0x00);
  i2cWrite(LIS2MDL_ADDR, CFG_REG_C, 0x00);
}

// Apply hard-iron and soft-iron corrections
void calibrateMag(float in[3], float out[3]) {
  float tmp[3] = {
    in[0] - hard_iron[0],
    in[1] - hard_iron[1],
    in[2] - hard_iron[2]
  };
  // matrix multiply: out = soft_iron * tmp
  for (int i = 0; i < 3; i++) {
    out[i] = soft_iron[i][0] * tmp[0]
           + soft_iron[i][1] * tmp[1]
           + soft_iron[i][2] * tmp[2];
  }
}

// Log one sample as CSV: t, ax,ay,az, gx,gy,gz, mx, my, mz (calibrated)
void logSample() {
  uint32_t t = millis();
  int16_t ax_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL);
  int16_t ay_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL+2);
  int16_t az_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL+4);
  int16_t gx_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_G);
  int16_t gy_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_G+2);
  int16_t gz_i = i2cRead16(LSM6DSL_ADDR, OUTX_L_G+4);
  int16_t mx_i = i2cRead16(LIS2MDL_ADDR, OUTX_L_M);
  int16_t my_i = i2cRead16(LIS2MDL_ADDR, OUTX_L_M+2);
  int16_t mz_i = i2cRead16(LIS2MDL_ADDR, OUTX_L_M+4);

  // Convert to float and calibrate magnetometer
  float m_in[3] = { (float)mx_i, (float)my_i, (float)mz_i };
  float m_out[3];
  calibrateMag(m_in, m_out);

  // Print calibrated values
  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f\n",
                t,
                ax_i, ay_i, az_i,
                gx_i, gy_i, gz_i,
                m_out[0], m_out[1], m_out[2]
  );
}

void setup() {
  Serial.begin(230400);
  while (!Serial) { delay(10); }
  Wire.begin(8, 10);

  // Initialize sensors in RAW mode
  initIMURaw();
  initMagRaw();
  delay(100);  // let settings settle

  // Print header & prompt
  Serial.println("t,ax,ay,az,gx,gy,gz,mx, my, mz");
  Serial.println("MODE,RAW");
  Serial.println("Waiting for 'S' to start logging...");

  // —— hand-shake: wait for 'S' from PC —— 
  while (true) {
    if (Serial.available()) {
      if (Serial.read() == 'S') break;
    }
    delay(10);
  }
  Serial.println("START");

  // Log for LOG_DURATION_MS
  uint32_t t0 = millis();
  while (millis() - t0 < LOG_DURATION_MS) {
    logSample();
    delay(10);   // ~100 Hz total loop
  }

  Serial.println("DONE");
}

void loop() {
  // nothing more to do
}
