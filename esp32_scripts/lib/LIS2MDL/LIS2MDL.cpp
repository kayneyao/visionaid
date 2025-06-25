#include "LIS2MDL.h"

#define LIS2MDL_REG_WHO_AM_I      0x4F
#define LIS2MDL_ID                0x40
#define LIS2MDL_REG_CFG_REG_A     0x60
#define LIS2MDL_REG_OUTX_L        0x68

#define SPI_READ_FLAG             0x80

bool LIS2MDL::beginI2C(TwoWire &wire, uint8_t addr) {
    _wire = &wire;
    _i2cAddr = addr;
    _useSPI = false;
    _wire->begin();
    if (readReg(LIS2MDL_REG_WHO_AM_I) != LIS2MDL_ID) return false;
    // default config: continuous mode
    writeReg(LIS2MDL_REG_CFG_REG_A, 0x10); // ODR=10Hz
    return true;
}

bool LIS2MDL::beginSPI(SPIClass &spi, int csPin) {
    _spi = &spi;
    _csPin = csPin;
    _useSPI = true;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
    if (readReg(LIS2MDL_REG_WHO_AM_I) != LIS2MDL_ID) return false;
    writeReg(LIS2MDL_REG_CFG_REG_A, 0x10);
    return true;
}

uint8_t LIS2MDL::readID() {
    return readReg(LIS2MDL_REG_WHO_AM_I);
}

void LIS2MDL::readMagnetometer(int16_t &mx, int16_t &my, int16_t &mz) {
    uint8_t buf[6];
    readRegs(LIS2MDL_REG_OUTX_L, buf, 6);
    mx = (int16_t)(buf[1] << 8 | buf[0]);
    my = (int16_t)(buf[3] << 8 | buf[2]);
    mz = (int16_t)(buf[5] << 8 | buf[4]);
}

void LIS2MDL::writeReg(uint8_t reg, uint8_t val) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(reg & ~SPI_READ_FLAG);
        _spi->transfer(val);
        digitalWrite(_csPin, HIGH);
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
}

uint8_t LIS2MDL::readReg(uint8_t reg) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(reg | SPI_READ_FLAG);
        uint8_t val = _spi->transfer(0x00);
        digitalWrite(_csPin, HIGH);
        return val;
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_i2cAddr, (uint8_t)1);
        return _wire->read();
    }
}

void LIS2MDL::readRegs(uint8_t startReg, uint8_t* buf, uint16_t len) {
    if (_useSPI) {
        digitalWrite(_csPin, LOW);
        _spi->transfer(startReg | SPI_READ_FLAG);
        for (uint16_t i = 0; i < len; i++) buf[i] = _spi->transfer(0x00);
        digitalWrite(_csPin, HIGH);
    } else {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(startReg);
        _wire->endTransmission(false);
        _wire->requestFrom(_i2cAddr, (uint8_t)len);
        for (uint16_t i = 0; i < len; i++) buf[i] = _wire->read();
    }
}
