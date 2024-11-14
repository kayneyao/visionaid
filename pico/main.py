from pico import MPU6050
from machine import I2C, UART, Pin
from time import sleep

imuI2C = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
IMU = MPU6050.MPU6050(i2c=imuI2C)

# # print(hex(imuI2C.scan()[0]).upper())
# print(IMU.who_am_i())
IMU.wake()

while True:
    print("Acc: " + str(IMU.read_accel_data()) +
          " Gyro: " + str(IMU.read_gyro_data()))
    # print(IMU.who_am_i())
    # print(i)
    # i+=1