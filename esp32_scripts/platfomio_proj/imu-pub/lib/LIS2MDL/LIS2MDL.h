#ifndef LIS2MDL_H
#define LIS2MDL_H

#include <Wire.h>
#include <SPI.h>

class LIS2MDL {
public:
    bool beginI2C(TwoWire &wire, uint8_t addr = 0x1E);

    // Read magnetic field in Tesla
    void readData(float &mx, float &my, float &mz, bool calib);

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

#endif // LIS2MDL_H