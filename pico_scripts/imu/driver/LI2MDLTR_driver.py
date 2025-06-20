'''
MicroPython driver for STMicroelectronics LIS2MDLTR 3-axis magnetometer.

Based on ST LIS2MDL datasheet Rev.6 and register map. Provides basic initialization, raw data reads, and conversion to µT and °C.

Sources:
- ST LIS2MDL datasheet Rev.6
- Adafruit CircuitPython LIS2MDL driver
'''

import time
import struct
from machine import I2C

class LIS2MDLTR:
    _ADDR = 0x1E  # 7-bit I2C address
    _WHO_AM_I = 0x4F
    _WHO_AM_I_RESPONSE = 0x40

    # Registers
    _CFG_REG_A = 0x60
    _CFG_REG_C = 0x62
    _STATUS    = 0x67
    _OUTX_L    = 0x68
    _OUTY_L    = 0x6A
    _OUTZ_L    = 0x6C
    _TEMP_OUT_L= 0x6E

    # Sensitivity
    _SENS_MGAUSS     = 1.5   # mgauss/LSB
    _MGAUSS_TO_UTESLA = 0.1  # µT per mgauss
    _SENS_UTESLA     = _SENS_MGAUSS * _MGAUSS_TO_UTESLA  # µT/LSB

    def __init__(self, i2c, addr=_ADDR):
        '''
        Initialize the LIS2MDLTR magnetometer over I2C.

        :param i2c: machine.I2C instance
        :param addr: I2C address (default 0x1E)
        '''
        self.i2c = i2c
        self.addr = addr

        # Verify device ID
        whoami = self._read_reg(self._WHO_AM_I)
        if whoami != self._WHO_AM_I_RESPONSE:
            raise OSError(f'LIS2MDLTR not found (WHO_AM_I=0x{whoami:02X})')
        time.sleep_ms(10)

        # Enable block data update
        self._write_reg(self._CFG_REG_C, 0x20)

        # Enable temp comp, high-res, ODR=10Hz, continuous mode
        self._write_reg(self._CFG_REG_A, 0x80)
        time.sleep_ms(10)

    def _write_reg(self, reg, value):
        '''Write a byte to register''' 
        self.i2c.writeto_mem(self.addr, reg, bytes([value]))

    def _read_reg(self, reg):
        '''Read a byte from register'''
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def read_raw(self):
        '''Read raw magnetometer data (X, Y, Z).'''
        data = self.i2c.readfrom_mem(self.addr, self._OUTX_L, 6)
        x, y, z = struct.unpack('<hhh', data)
        return x, y, z

    def read_magnetic(self):
        '''Read magnetic field in µT.'''
        x, y, z = self.read_raw()
        return (x * self._SENS_UTESLA, y * self._SENS_UTESLA, z * self._SENS_UTESLA)

    def read_raw_temperature(self):
        '''Read raw temperature output.'''
        data = self.i2c.readfrom_mem(self.addr, self._TEMP_OUT_L, 2)
        return struct.unpack('<h', data)[0]

    def read_temperature(self):
        '''Read temperature in °C (1 LSB = 1/8 °C).'''
        raw = self.read_raw_temperature()
        return raw / 8.0

    def data_ready(self):
        '''Check if new data is available'''
        status = self._read_reg(self._STATUS)
        return bool(status & 0x08)
