import serial
import pynmea2
import matplotlib.pyplot as plt
import numpy as np
# import RPi.GPIO as GPIO

class Sensors(object):
    t = 0 
    iState = False
    
    soft = np.array([[1.414817, -0.016987, 0.064807],
                [-0.016987, 1.349957, -0.019416],
                [0.064807, -0.019416, 1.065155]])
    hard = np.array([0.344155, -14.308739, -14.832437])
        
    A = np.array([[1.021179, -0.042233, -0.000608],  # 'A^-1' matrix from Magneto
              [-0.042233, 1.022789, -0.007579],
              [-0.000608, -0.007579, 1.108518]])
# 'Combined bias (b)' vector from Magneto
    b = np.array([-0.011276, 0.014394, -0.049340])
    
    
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)
        
        self.output = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],0]
        
        self.pitches = []
        self.rolls = []
        self.yaws = []
        
        self.iT = 0
        
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        
        self.fig , self.ax = plt.subplots(nrows=3, sharex=True, gridspec_kw={"height_ratios": [3,3,3]})
        plt.ion()
        plt.show()
        
        # GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(26, GPIO.OUT)

        # GPIO.output(26, GPIO.HIGH)
    
            
    def getValues(self):
        data = self.ser.readline().decode().split(", ")
        
        if data.__len__() != 13:
            return self.output
        
        mag = list(map(float, data[0:3]))
        gyro = list(map(float, data[3:6]))
        acc = list(map(float, data[6:9]))
        gps = list(map(float, data[9:12]))
        time = int(data[12])
        
        mag = self.soft @ (np.array(mag) - self.hard)
        
        acc = self.A @ (np.array(acc) - self.b)
        
        self.output = [mag, gyro, acc, gps, time]
        
        return self.output
        
    # def absAcc(self, dT):
    #     imu = self.getIMU()
    #     comp = imu[0]
    #     mag = imu[1]
    #     #angle relative to 0 = gyro*dT
    #     rot += tuple(tmp * dT for tmp in comp)
    #     #linear velo relative to IMU orientation
    #     vel += 
        
    def PPS(self):
        cState = GPIO.input(26)
        
        if cState > iState:
            t = t + 1
        iState = cState
           
        return t
    
    def gyroOrien(self):
        Z = self.getValues()
        x = Z[1][0]
        y = Z[1][1]
        z = Z[1][2]
        time = Z[4]
        dT = time - self.iT
        
        self.roll += x * dT * 0.001
        self.pitch += y * dT * 0.001
        self.yaw += z * dT * 0.001
    
        self.iT = time
        
        print([self.yaw, self.pitch, self.roll])
        
        
        # if(self.pitches.__len__() > 20):
        #     self.pitches.pop(0)
        #     self.yaws.pop(0)
        #     self.rolls.pop(0)
        # self.pitches.append(self.pitch)
        # self.yaws.append(self.yaw)
        # self.rolls.append(self.roll)
        # self.ax[0].clear()
        # self.ax[1].clear()
        
        # self.ax[2].clear()
        # self.ax[0].plot(self.pitches, "tab:red", label="Pitch")
        # self.ax[1].plot(self.rolls, "tab:green", label="Roll")
        # self.ax[2].plot(self.yaws, "tab:blue", label="Yaw")
        # self.ax[0].grid()
        # self.ax[0].legend()
        # self.ax[1].grid()
        # self.ax[1].legend()
        
        # self.ax[2].grid()
        # self.ax[2].legend()
    
    def createSystem(self):
        self.system = Sensors()
    
    def getSystem(self):
        return self.system
    
sens = Sensors()

while 1:
    sens.gyroOrien()