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
static constexpr float GYR_SENS = 8.75e-3f * (M_PI/180.0f);  // mdps/LSB → rad/s

bool LSM6DSL::beginI2C(TwoWire &wire, uint8_t addr) {
    _wire = &wire;
    _i2cAddr = addr;
    _useSPI = false;
    _wire->begin();
    if (readReg(REG_WHO_AM_I) != WHO_AM_I_ID) return false;

    
    writeReg(REG_CTRL1_XL, 0x42);
    writeReg(REG_CTRL2_G,  0x40 | 0x00);
    writeReg(REG_CTRL3_C, 0x44 | 0x04);
    writeReg(REG_CTRL5_C, 0x00);
    writeReg(REG_CTRL8_XL, 0x10)
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
                       float &gx, float &gy, float &gz) {
    uint8_t buf[6];
    // accel
    readRegs(REG_OUTX_L_XL, buf, 6);  
    int16_t rx = int16_t(buf[0] | (buf[1] << 8));
    int16_t ry = int16_t(buf[2] | (buf[3] << 8));
    int16_t rz = int16_t(buf[4] | (buf[5] << 8));
    // ax = rx * ACC_SENS;
    // ay = ry * ACC_SENS;
    // az = rz * ACC_SENS;
    ax = -ry * ACC_SENS;
    ay = rx * ACC_SENS;
    az = rz * ACC_SENS;

    // gyro
    readRegs(REG_OUTX_L_G, buf, 6);
    int16_t gx_raw = int16_t(buf[0] | (buf[1] << 8));
    int16_t gy_raw = int16_t(buf[2] | (buf[3] << 8));
    int16_t gz_raw = int16_t(buf[4] | (buf[5] << 8));
    // gx = gx_raw * GYR_SENS;
    // gy = gy_raw * GYR_SENS;
    // gz = gz_raw * GYR_SENS;
    gx = -gy_raw * GYR_SENS;
    gy = gx_raw * GYR_SENS;
    gz = gz_raw * GYR_SENS;
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