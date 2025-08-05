#include "LIS2MDL.h"

#define REG_WHO_AM_I   0x4F
#define WHO_AM_I_ID    0x40
#define REG_CFG_A      0x60
#define REG_CFG_B      0x61
#define REG_CFG_C      0x62
#define REG_STATUS     0x67
#define REG_OUTX_L     0x68

static constexpr float MAG_SENS = 1.5e-1f;

const float hard_iron[3] = {
  2.82, -1.33, -2.60
};

const float soft_iron[3][3] = {
  {0.981, 0.017, -0.012},
  {0.017, 1.015, 0.014},
  {-0.012, 0.014, 1.006}
};

bool LIS2MDL::beginI2C(TwoWire &wire, uint8_t addr) {
    _wire = &wire;
    _i2cAddr = addr;
    _useSPI = false;
    _wire->begin();
    if (readReg(REG_WHO_AM_I) != WHO_AM_I_ID) return false;
    writeReg(REG_CFG_A, 0x8C);
    writeReg(REG_CFG_B, 0x03);
    writeReg(REG_CFG_C, 0x10);
    return true;
}

void LIS2MDL::readData(float &mx, float &my, float &mz, bool calib) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(REG_OUTX_L);
    Wire.endTransmission(false);
    Wire.requestFrom(_i2cAddr, 6);

    int16_t rawX = int16_t(Wire.read() | (Wire.read() << 8));
    int16_t rawY = int16_t(Wire.read() | (Wire.read() << 8));
    int16_t rawZ = int16_t(Wire.read() | (Wire.read() << 8));
    mx = rawX * MAG_SENS;
    my = rawY * MAG_SENS;
    mz = rawZ * MAG_SENS;

    if(calib){
        float hcal_mx, hcal_my, hcal_mz;

        float mag_data[3]; 

        hcal_mx = mx - hard_iron[0];
        hcal_my = my - hard_iron[1];
        hcal_mz = mz - hard_iron[2];

        for(int i = 0; i < 3; i++){
          mag_data[i] = (soft_iron[i][0] * hcal_mx) +
                        (soft_iron[i][1] * hcal_my) +
                        (soft_iron[i][2] * hcal_mz);
        }

        mx = mag_data[0] * 1e-6;
        my = -mag_data[1] * 1e-6;
        mz = mag_data[2] * 1e-6;
    }


}

void LIS2MDL::writeReg(uint8_t reg, uint8_t val) {
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

uint8_t LIS2MDL::readReg(uint8_t reg) {
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

void LIS2MDL::readRegs(uint8_t startReg, uint8_t* buf, uint16_t len) {
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