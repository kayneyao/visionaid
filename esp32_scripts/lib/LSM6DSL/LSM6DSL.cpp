#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <Wire.h>
#include <SPI.h>

class LSM6DSL {
public:
    // Initialize over I2C (addr = 0x6A or 0x6B)
    bool beginI2C(TwoWire &wire, uint8_t addr = 0x6A);
    // Initialize over SPI (csPin = chip-select GPIO)
    bool beginSPI(SPIClass &spi, int csPin);

    // Read sensor ID (WHO_AM_I)
    uint8_t readID();

    // Read raw measurements
    void readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az);
    void readGyroscope(int16_t &gx, int16_t &gy, int16_t &gz);

private:
    TwoWire* _wire = nullptr;
    SPIClass* _spi = nullptr;
    int8_t _csPin = -1;
    uint8_t _i2cAddr = 0;
    bool _useSPI = false;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t startReg, uint8_t* buf, uint16_t len);
};

#endif