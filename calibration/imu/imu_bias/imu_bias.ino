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

// helper: write one byte to a reg
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

// -- INITIALIZATION MODES --

void initIMURaw() {
  // Basic 104 Hz, ±2 g / ±245 dps, filters OFF
  i2cWrite(LSM6DSL_ADDR, CTRL1_XL, 0x40);  // ODR_XL=104Hz, FS_XL=±2 g
  i2cWrite(LSM6DSL_ADDR, CTRL2_G,  0x40);  // ODR_G=104Hz, FS_G=±245 dps
  i2cWrite(LSM6DSL_ADDR, CTRL8_XL, 0x00);  // LPF2 off, composite off
  i2cWrite(LSM6DSL_ADDR, CTRL5_C,  0x00);  // self-test off
}

void initMagRaw() {
  // 100 Hz continuous-mode, but no LPF / no offset-cancel / no BDU
  i2cWrite(LIS2MDL_ADDR, CFG_REG_A, 0x80 | 0x0C);  
    // COMP_TEMP_EN=1, DO=100Hz (bits7+2:0b10001100), MD=00
  i2cWrite(LIS2MDL_ADDR, CFG_REG_B, 0x00);  
    // no offset-cancel, no LPF
  i2cWrite(LIS2MDL_ADDR, CFG_REG_C, 0x00);  
    // BDU=0, self-test=0
}

void initIMUFiltered() {
  // 104 Hz, ±2 g/±245 dps + LPF1+LPF2 enabled
  i2cWrite(LSM6DSL_ADDR, CTRL1_XL, 0x40 | 0x02);  // set LPF1_BW_SEL=1
  i2cWrite(LSM6DSL_ADDR, CTRL8_XL, 0x10);         // LPF2_XL_EN=1
  i2cWrite(LSM6DSL_ADDR, CTRL5_C,  0x00);         // no self-test
}

void initMagFiltered() {
  // 100 Hz, enable temp-comp, LPF, offset-cancel, BDU
  i2cWrite(LIS2MDL_ADDR, CFG_REG_A, 0x80 | 0x0C);  // temp-comp, DO=100Hz
  i2cWrite(LIS2MDL_ADDR, CFG_REG_B, 0x06);         // OFF_CANC=1, LPF=1
  i2cWrite(LIS2MDL_ADDR, CFG_REG_C, 0x12);         // BDU=1, self-test=1
}

// -- LOG ONE SAMPLE (CSV) --

void logSample() {
  uint32_t t = millis();
  int16_t ax = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL);
  int16_t ay = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL+2);
  int16_t az = i2cRead16(LSM6DSL_ADDR, OUTX_L_XL+4);
  int16_t gx = i2cRead16(LSM6DSL_ADDR, OUTX_L_G);
  int16_t gy = i2cRead16(LSM6DSL_ADDR, OUTX_L_G+2);
  int16_t gz = i2cRead16(LSM6DSL_ADDR, OUTX_L_G+4);
  int16_t mx = i2cRead16(LIS2MDL_ADDR, OUTX_L_M);
  int16_t my = i2cRead16(LIS2MDL_ADDR, OUTX_L_M+2);
  int16_t mz = i2cRead16(LIS2MDL_ADDR, OUTX_L_M+4);

  // CSV: time, ax,ay,az, gx,gy,gz, mx,my,mz
  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                t, ax, ay, az, gx, gy, gz, mx, my, mz);
}

void setup() {
  Serial.begin(230400);
  Wire.begin(8, 10);

  // 1) RAW mode
  initIMURaw();
  initMagRaw();
  delay(100);               // let filters settle

  delay(10000);

  Serial.println("MODE,RAW");
  uint32_t t0 = millis();
  while (millis() - t0 < 300000UL) { // 300 000 ms = 5 min
    logSample();
    delay(10);            // ~100 Hz overall loop
  }

  // 2) FILTERED mode
  initIMUFiltered();
  initMagFiltered();
  delay(100);

  Serial.println("MODE,FILTERED");
  t0 = millis();
  while (millis() - t0 < 300000UL) {
    logSample();
    delay(10);
  }

  Serial.println("DONE");
}

void loop() {
  // nothing
}
