"""
main.py - example script to initialize LSM6DSLTR and LIS2MDLTR sensors
and print their readings over I2C in MicroPython.

Adjust I2C pins and bus number as needed for your board.
"""

import time
from machine import I2C, Pin

# Import sensor drivers
from driver import LSM6DSLTR
from driver import LIS2MDLTR

# Initialize I2C bus (example for Raspberry Pi Pico: I2C0 on GP0=SDA, GP1=SCL)
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

# Initialize sensors
try:
    accelgyro = LSM6DSLTR(i2c)
    print("LSM6DSLTR initialized successfully.")
except OSError as e:
    print("Failed to initialize LSM6DSLTR:", e)
    raise

try:
    magnetometer = LIS2MDLTR(i2c)
    print("LIS2MDLTR initialized successfully.")
except OSError as e:
    print("Failed to initialize LIS2MDLTR:", e)
    raise

# Main loop: read and print sensor values
while True:
    # Read accelerometer (m/s^2) and gyroscope (°/s)
    ax, ay, az = accelgyro.read_acceleration()
    gx, gy, gz = accelgyro.read_gyro()

    # Read magnetometer (µT)
    mx, my, mz = magnetometer.read_magnetic()

    # Read temperatures
    temp_ag = accelgyro.read_temperature()
    temp_mag = magnetometer.read_temperature()

    # Print readings
    print("Accelerometer (m/s^2): X={:.2f}, Y={:.2f}, Z={:.2f}".format(ax, ay, az))
    print("Gyroscope (°/s):        X={:.2f}, Y={:.2f}, Z={:.2f}".format(gx, gy, gz))
    print("Magnetometer (µT):      X={:.2f}, Y={:.2f}, Z={:.2f}".format(mx, my, mz))
    print("Temp (Acc/Gyro): {:.2f} °C".format(temp_ag))
    print("Temp (Magnetometer): {:.2f} °C".format(temp_mag))
    print("-------------------------------")

    # Wait 1 second between readings
    time.sleep(1)
