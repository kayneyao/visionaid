#include <Wire.h>

// ——— I²C & LSM6DSL definitions —————————————————————————
#define SDA_PIN            21          // change to your wiring
#define SCL_PIN            22
#define LSM6DSL_ADDR       0x6A        // or 0x6B if SA0 is pulled HIGH

// Control registers
#define REG_CTRL1_XL       0x10  // accel config
#define REG_CTRL2_G        0x11  // gyro config
#define REG_CTRL3_C        0x12  // common settings (IF_INC)
 
// Data output registers (auto-increment if IF_INC=1)
#define REG_OUTX_L_G       0x22
#define REG_OUTX_L_XL      0x28

// Settings values
#define XL_ODR_104HZ       0x40  // 104 Hz accel
#define XL_FS_2G           0x00  // ±2 g
#define G_ODR_104HZ        0x40  // 104 Hz gyro
#define G_FS_250DPS        0x00  // ±250 dps
#define IF_INC_ENABLED     0x04  // auto-increment registers

// Sensitivity conversions
const float ACCEL_SENS = 0.061e-3;   // 0.061 mg/LSB @ ±2 g → 0.000061 g/LSB
const float GYRO_SENS  = 8.75e-3;    // 8.75 mdps/LSB @ ±250 dps → 0.00875 dps/LSB

// ——— Low-level I²C helpers ——————————————————————————————
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSL_ADDR, (uint8_t)1);
  return Wire.read();
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSL_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

// ——— Setup & Loop ——————————————————————————————————————
void setup() {
  Serial.begin(115200);
  while (!Serial) { }  
  Serial.println("LSM6DSL I2C Data Demo");

  // init I²C on ESP32
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  // enable auto-increment on multi-byte registers
  writeReg(REG_CTRL3_C, IF_INC_ENABLED);

  // configure accel: ODR=104 Hz, ±2 g
  writeReg(REG_CTRL1_XL, XL_ODR_104HZ | XL_FS_2G);

  // configure gyro:  ODR=104 Hz, ±250 dps
  writeReg(REG_CTRL2_G, G_ODR_104HZ  | G_FS_250DPS);

  delay(100);
}

void loop() {
  uint8_t raw[12];
  // read 6 gyro bytes starting at OUTX_L_G, then 6 accel bytes at OUTX_L_XL
  readRegs(REG_OUTX_L_G, raw, 12);

  // gyro
  int16_t gx = (int16_t)(raw[1] << 8 | raw[0]);
  int16_t gy = (int16_t)(raw[3] << 8 | raw[2]);
  int16_t gz = (int16_t)(raw[5] << 8 | raw[4]);

  // accel
  int16_t ax = (int16_t)(raw[7]  << 8 | raw[6]);
  int16_t ay = (int16_t)(raw[9]  << 8 | raw[8]);
  int16_t az = (int16_t)(raw[11] << 8 | raw[10]);

  // convert to human units
  float fgx = gx * GYRO_SENS;
  float fgy = gy * GYRO_SENS;
  float fgz = gz * GYRO_SENS;
  float fax = ax * ACCEL_SENS * 9.80665; // g→m/s²
  float fay = ay * ACCEL_SENS * 9.80665;
  float faz = az * ACCEL_SENS * 9.80665;

  // print
  Serial.print("Gyro [dps]: ");
  Serial.print(fgx, 2); Serial.print(", ");
  Serial.print(fgy, 2); Serial.print(", ");
  Serial.println(fgz, 2);

  Serial.print("Accel [m/s²]: ");
  Serial.print(fax, 2); Serial.print(", ");
  Serial.print(fay, 2); Serial.print(", ");
  Serial.println(faz, 2);

  Serial.println();
  delay(200);
}
