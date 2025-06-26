#include "LSM6DSL.h"

// Register definitions
#define LSM6DSL_REG_WHO_AM_I      0x0F
#define LSM6DSL_ID                0x6A
#define LSM6DSL_REG_CTRL1_XL       0x10  // accel config
#define LSM6DSL_REG_CTRL2_G        0x11  // gyro config
#define LSM6DSL_REG_CTRL3_C       0x12
#define LSM6DSL_IF_INC            0x04  // enable auto-increment
#define LSM6DSL_REG_OUTX_L_G      0x22
#define LSM6DSL_REG_OUTX_L_XL     0x28

// Settings values
#define XL_ODR_104HZ       0x40  // 104 Hz accel
#define XL_FS_2G           0x00  // ±2 g
#define G_ODR_104HZ        0x40  // 104 Hz gyro
#define G_FS_250DPS        0x00  // ±250 dps
#define IF_INC_ENABLED     0x04  // auto-increment registers

// SPI flags
#define SPI_READ_FLAG             0x80

bool LSM6DSL::beginI2C(TwoWire &wire, uint8_t addr) {
    _wire = &wire;
    _i2cAddr = addr;
    _useSPI = false;
    _wire->begin();
    // Verify ID
    if (readReg(LSM6DSL_REG_WHO_AM_I) != LSM6DSL_ID) return false;
    
    // enable auto-increment on multi-byte registers
    writeReg(LSM6DSL_REG_CTRL3_C, IF_INC_ENABLED);

    // configure accel: ODR=104 Hz, ±2 g
    writeReg(LSM6DSL_REG_CTRL1_XL, XL_ODR_104HZ | XL_FS_2G);

    // configure gyro:  ODR=104 Hz, ±250 dps
    writeReg(LSM6DSL_REG_CTRL2_G, G_ODR_104HZ  | G_FS_250DPS);

    

    return true;
}

bool LSM6DSL::beginSPI(SPIClass &spi, int csPin) {
    _spi = &spi;
    _csPin = csPin;
    _useSPI = true;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
    // Verify ID
    if (readReg(LSM6DSL_REG_WHO_AM_I) != LSM6DSL_ID) return false;
    // Enable auto-increment
    writeReg(LSM6DSL_REG_CTRL3_C, LSM6DSL_IF_INC);
    return true;
}

uint8_t LSM6DSL::readID() {
    return readReg(LSM6DSL_REG_WHO_AM_I);
}

void LSM6DSL::readData(int16_t &ax, int16_t &ay, int16_t &az,
                       int16_t &gx, int16_t &gy, int16_t &gz){
    uint8_t buf[6];

    readRegs(LSM6DSL_REG_OUTX_L_G, buf, 6);

    gx = (int16_t)(buf[1] << 8 | buf[0]);
    gy = (int16_t)(buf[3] << 8 | buf[2]);
    gz = (int16_t)(buf[5] << 8 | buf[4]);

    readRegs(LSM6DSL_REG_OUTX_L_XL, buf, 6);

    ax = (int16_t)(buf[1]  << 8 | buf[0]);
    ay = (int16_t)(buf[3]  << 8 | buf[2]);
    az = (int16_t)(buf[5] << 8 | buf[4]);
}

void LSM6DSL::writeReg(uint8_t reg, uint8_t val) {
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

uint8_t LSM6DSL::readReg(uint8_t reg) {
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

void LSM6DSL::readRegs(uint8_t startReg, uint8_t* buf, uint16_t len) {
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