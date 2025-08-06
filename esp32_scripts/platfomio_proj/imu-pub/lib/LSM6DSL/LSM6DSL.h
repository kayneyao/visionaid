#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <Wire.h>
#include <SPI.h>

class LSM6DSL {
public:
    bool beginI2C(TwoWire &wire, uint8_t addr = 0x6A);
    bool beginSPI(SPIClass &spi, int csPin);

    // Read accelerometer (m/s²) and gyroscope (rad/s)
    void readData(float &ax, float &ay, float &az,
                  float &gx, float &gy, float &gz,
                  bool calib);

private:
    TwoWire*  _wire = nullptr;
    SPIClass* _spi  = nullptr;
    int8_t    _csPin = -1;
    uint8_t   _i2cAddr = 0;
    bool      _useSPI  = false;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t startReg, uint8_t* buf, uint16_t len);
};

#endif // LSM6DSL_H