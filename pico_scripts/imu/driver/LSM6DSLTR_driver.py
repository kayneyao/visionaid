"""
MicroPython driver for STMicroelectronics LSM6DSLTR 6-axis IMU sensor.

This driver is based on the ST LSM6DSL datasheet and register map. It provides basic initialization,
raw data reads, and conversion to SI units for accelerometer, gyroscope, and temperature measurements.

Sources:
- ST LSM6DSL datasheet (Rev.5) ([st.com](https://www.st.com/resource/en/datasheet/lsm6dsl.pdf?utm_source=chatgpt.com))
- Adafruit CircuitPython LSM6DSOX driver documentation ([docs.circuitpython.org](https://docs.circuitpython.org/projects/lsm6dsox/en/latest/?utm_source=chatgpt.com))
"""

import time
import struct
from machine import I2C


class LSM6DSLTR:
    _ADDR = 0x6A  # default I2C address for LSM6DSLTR

    # Register addresses
    _WHO_AM_I = 0x0F
    _WHO_AM_I_RESPONSE = 0x6A
    _CTRL1_XL = 0x10
    _CTRL2_G = 0x11
    _CTRL3_C = 0x12
    _OUT_TEMP_L = 0x20
    _OUT_TEMP_H = 0x21
    _STATUS = 0x1E
    _OUTX_L_G = 0x22
    _OUTX_H_G = 0x23
    _OUTY_L_G = 0x24
    _OUTY_H_G = 0x25
    _OUTZ_L_G = 0x26
    _OUTZ_H_G = 0x27
    _OUTX_L_XL = 0x28
    _OUTX_H_XL = 0x29
    _OUTY_L_XL = 0x2A
    _OUTY_H_XL = 0x2B
    _OUTZ_L_XL = 0x2C
    _OUTZ_H_XL = 0x2D

    # Sensor sensitivities
    _ACCEL_SENSITIVITY = 0.061  # mg/LSB for ±2g
    _GYRO_SENSITIVITY = 8.75    # mdps/LSB for ±245 dps

    def __init__(self, i2c: I2C, addr: int = _ADDR):
        """
        Initialize the sensor over I2C.

        :param i2c: Initialized machine.I2C object
        :param addr: I2C address (default 0x6A)
        """
        self.i2c = i2c
        self.addr = addr

        # Verify device identity
        whoami = self._read_reg(self._WHO_AM_I)
        if whoami != self._WHO_AM_I_RESPONSE:
            raise OSError(f"LSM6DSLTR not found (WHO_AM_I=0x{whoami:02X})")
        time.sleep_ms(10)

        # Enable Block Data Update (BDU) and auto-increment of register address (IF_INC)
        self._write_reg(self._CTRL3_C, 0x44)

        # Configure accelerometer: 104 Hz, ±2g, BW = 100 Hz
        # ODR_XL = 0b0100 << 4 = 0x40, FS_XL = ±2g (0b00 << 2)
        self._write_reg(self._CTRL1_XL, 0x40)

        # Configure gyroscope: 104 Hz, ±245 dps
        # ODR_G = 0b0100 << 4 = 0x40, FS_G = 245 dps (0b00 << 2)
        self._write_reg(self._CTRL2_G, 0x40)

    def _write_reg(self, reg: int, value: int) -> None:
        """Write a byte to the given register."""
        self.i2c.writeto_mem(self.addr, reg, bytes([value]))

    def _read_reg(self, reg: int) -> int:
        """Read a single byte from the given register."""
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def read_raw_accel(self) -> tuple[int, int, int]:
        """Read raw accelerometer data (X, Y, Z)."""
        data = self.i2c.readfrom_mem(self.addr, self._OUTX_L_XL, 6)
        x, y, z = struct.unpack('<hhh', data)
        return x, y, z

    def read_raw_gyro(self) -> tuple[int, int, int]:
        """Read raw gyroscope data (X, Y, Z)."""
        data = self.i2c.readfrom_mem(self.addr, self._OUTX_L_G, 6)
        x, y, z = struct.unpack('<hhh', data)
        return x, y, z

    def read_acceleration(self) -> tuple[float, float, float]:
        """Read acceleration in m/s²."""
        raw = self.read_raw_accel()
        # Convert: raw * sensitivity (mg/LSB) * 1e-3 (g) * 9.80665 (m/s²)
        return tuple(v * self._ACCEL_SENSITIVITY * 1e-3 * 9.80665 for v in raw)

    def read_gyro(self) -> tuple[float, float, float]:
        """Read angular rate in degrees per second."""
        raw = self.read_raw_gyro()
        # Convert: raw * sensitivity (mdps/LSB) * 1e-3 (dps)
        return tuple(v * self._GYRO_SENSITIVITY * 1e-3 for v in raw)

    def read_temperature(self) -> float:
        """Read temperature in °C."""
        data = self.i2c.readfrom_mem(self.addr, self._OUT_TEMP_L, 2)
        temp_raw = struct.unpack('<h', data)[0]
        # Datasheet: TEMP (°C) = 25 + (temp_raw / 16)
        return 25.0 + temp_raw / 16.0

    def data_ready(self) -> bool:
        """Check if new accel or gyro data is available."""
        status = self._read_reg(self._STATUS)
        # STATUS_REG bit 0: XLDA, bit 1: GDA
        return bool(status & 0x03)
