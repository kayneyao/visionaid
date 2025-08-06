#include "LSM6DSL.h"
#include <cmath>

#define REG_WHO_AM_I   0x0F
#define WHO_AM_I_ID    0x6A
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_CTRL5_C    0x00
#define REG_CTRL8_XL   0x17
#define REG_OUTX_L_G   0x22
#define REG_OUTX_L_XL  0x28

static constexpr float ACC_SENS = 0.061e-3f * 9.80665f;      // mg/LSB → m/s²
static constexpr float GYR_SENS = 1.750e-2f * (M_PI/180.0f);  // mdps/LSB → rad/s

const float offset_b[3] = {
  0.112260, -0.110812, 0.330895
};

const float offset_A[3][3] = {
  {0.997626, -0.004172, 0.000953},
  {-0.004172, 0.991214, -0.004644},
  {0.000953, -0.004644, 0.991372}
};

bool LSM6DSL::beginI2C(TwoWire &wire, uint8_t addr) {
    _wire = &wire;
    _i2cAddr = addr;
    _useSPI = false;
    _wire->begin();
    if (readReg(REG_WHO_AM_I) != WHO_AM_I_ID) return false;

    
    writeReg(REG_CTRL1_XL, 0x42);
    writeReg(REG_CTRL2_G,  0x44);
    writeReg(REG_CTRL3_C, 0x44 | 0x04);
    writeReg(REG_CTRL5_C, 0x00);
    writeReg(REG_CTRL8_XL, 0x10);
    return true;
}

bool LSM6DSL::beginSPI(SPIClass &spi, int csPin) {
    _spi = &spi;
    _csPin = csPin;
    _useSPI = true;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
    if (readReg(REG_WHO_AM_I) != WHO_AM_I_ID) return false;

    writeReg(REG_CTRL3_C, 0x44 | 0x04);
    writeReg(REG_CTRL1_XL, 0x40 | 0x00);
    writeReg(REG_CTRL2_G,  0x40 | 0x00);
    return true;
}

void LSM6DSL::readData(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz,
                       bool calib) {
    uint8_t buf[6];
    // accel
    readRegs(REG_OUTX_L_XL, buf, 6);  
    int16_t rx = int16_t(buf[0] | (buf[1] << 8));
    int16_t ry = int16_t(buf[2] | (buf[3] << 8));
    int16_t rz = int16_t(buf[4] | (buf[5] << 8));
    ax = -ry * ACC_SENS;
    ay = rx * ACC_SENS;
    az = rz * ACC_SENS;
    

    if(calib){
        float cal_ax, cal_ay, cal_az;

        float acc_data[3]; 

        cal_ax = ax - offset_b[0];
        cal_ay = ay - offset_b[1];
        cal_az = az - offset_b[2];

        for(int i = 0; i < 3; i++){
          acc_data[i] = (offset_A[i][0] * cal_ax) +
                        (offset_A[i][1] * cal_ay) +
                        (offset_A[i][2] * cal_az);
        }

        ax = acc_data[0];
        ay = acc_data[1];
        az = acc_data[2];
    }

    // gyro
    readRegs(REG_OUTX_L_G, buf, 6);
    int16_t gx_raw = int16_t(buf[0] | (buf[1] << 8));
    int16_t gy_raw = int16_t(buf[2] | (buf[3] << 8));
    int16_t gz_raw = int16_t(buf[4] | (buf[5] << 8));
    // gx = gx_raw * GYR_SENS;
    // gy = gy_raw * GYR_SENS;
    // gz = gz_raw * GYR_SENS;
    gx = -gy_raw * GYR_SENS - 0.05f;
    gy = gx_raw * GYR_SENS;
    gz = gz_raw * GYR_SENS - 0.016f;
}

void LSM6DSL::writeReg(uint8_t reg, uint8_t val) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(reg & 0x7F);
        _spi->transfer(val);
        digitalWrite(_csPin, HIGH);
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
}

uint8_t LSM6DSL::readReg(uint8_t reg) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(reg | 0x80);
        uint8_t v = _spi->transfer(0x00);
        digitalWrite(_csPin, HIGH);
        return v;
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_i2cAddr, (uint8_t)1);
        return _wire->read();
    }
}

void LSM6DSL::readRegs(uint8_t startReg, uint8_t* buf, uint16_t len) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(startReg);
        for (uint16_t i = 0; i < len; ++i) buf[i] = _spi->transfer(0x00);
        digitalWrite(_csPin, HIGH);
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(startReg);
        _wire->endTransmission(false);
        _wire->requestFrom(_i2cAddr, (uint8_t)len);
        for (uint16_t i = 0; i < len; ++i) buf[i] = _wire->read();
    }
}