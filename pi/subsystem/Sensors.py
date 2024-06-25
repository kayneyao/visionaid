import serial
from time import sleep
import pynmea2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from scipy import signal
import sys
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
        sleep(2)
        
        self.output = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],0,0]
        
        self.pitches = []
        self.rolls = []
        self.yaws = []
        
        self.iT = self.getValues()[4]
        
        self.posX = []
        self.posY = []
        
        self.dx = 0
        self.dy = 0
        

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        
        self.fig , self.ax = plt.subplots(nrows=3, sharex=True, gridspec_kw={"height_ratios": [6,1,1]})
        # plt.ion()
        # plt.show()
        
        # GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(26, GPIO.OUT)

        # GPIO.output(26, GPIO.HIGH)
    
            
    def getValues(self):
        # self.ser.write(b'g')
        try:
            self.ser.write(b'g')
            data = self.ser.readline().decode().split(", ")
            
            if data.__len__() != 13:
                return self.output
            
            mag = list(map(float, data[0:3]))
            gyro = list(map(float, data[3:6]))
            acc = list(map(float, data[6:9]))
            gps = list(map(float, data[9:12]))
            time = int(data[12])
            
            mag = self.soft @ (np.array(mag) - self.hard)
            
            heading = np.arctan2(mag[1], mag[0]) - 4.9 * np.pi/180
            
            if(heading < 0):
                heading += 2*np.pi
            elif(heading > 2*np.pi):
                heading -= 2*np.pi
            
            acc = self.A @ (np.array(acc) - self.b)

            self.output = [mag, gyro, acc, gps, time, heading]
            
            return self.output
        except:
            pass
        
        return self.output
        
        
    
    def odomBasic(self, i):
        Z = self.getValues()
        
        cT = Z[4]
        
        x = Z[2][0] - 1
        y = Z[2][1] + 0.3
        
        dT = cT - self.iT
        
        self.iT = cT
        
        heading = Z[5]
        
        xa = x * np.cos(-heading) - y * np.sin(-heading)
        ya = x * np.sin(-heading) + y * np.cos(-heading)
        
        self.dx += xa * (dT/1000)**2
        self.dy += ya * (dT/1000)**2
        
        self.posX.append(self.dx)
        self.posY.append(self.dy)
        
        print(self.dx, self.dy)
        
        self.ax[0].clear()
        self.ax[0].plot(self.posX[-10:], self.posY[-10:])
        self.ax[0].set_xlim(-0.5, 0.5)
        self.ax[0].set_ylim(-0.5, 0.5)

        self.ax[0].set_title("Orientation")
        self.ax[0].set_ylabel("Y-Axis")
        self.ax[0].set_xlabel("X-Axis")
        
        
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
        Z = np.genfromtxt("./pi/subsystem/sensor.csv", delimiter=",", skip_header=1)
        x = Z[:498, 1]
        y = Z[:498, 2]
        z = Z[:498, 3]
        iT = Z[:, 0]
        
        cT = np.delete(iT, 0)
        iT = np.delete(iT, 498)
        
        dT = cT - iT
        
        # print([x, y, z, time])
        
        for i in range(498):
            self.roll += ((x).reshape(498,)[i] * 0.001 + 0.00355482) * dT[i] 
            self.pitch += ((y).reshape(498,)[i] * 0.001 - 0.00420793) * dT[i]
            self.yaw += ((z).reshape(498,)[i] * -0.001 + 0.00026876) * dT[i]
            # self.roll += ((x).reshape(498,)[i] * 0.001) * dT[i] 
            # self.pitch += ((y).reshape(498,)[i] * 0.001) * dT[i]
            # self.yaw += ((z).reshape(498,)[i] * -0.001) * dT[i]
            self.pitches.append(self.pitch)
            self.yaws.append(self.yaw)
            self.rolls.append(self.roll)
        
        # return [self.yaw, self.pitch, self.roll]
        
        # if(self.pitches.__len__() > 100):
        #     self.pitches.pop(0)
        #     self.yaws.pop(0)
        #     self.rolls.pop(0)
        # self.pitches.append(self.pitch)
        # self.yaws.append(self.yaw)
        # self.rolls.append(self.roll)
        
        offx = (self.roll - self.rolls[0])/(self.rolls.__len__()*dT)
        offy = (self.pitch - self.pitches[0])/(self.pitches.__len__()*dT)
        offz = (self.yaw - self.yaws[0])/(self.yaws.__len__()*dT)
        
        # self.ax[0].clear()
        # self.ax[1].clear()
        
        # self.ax[2].clear()
        self.ax[0].plot(cT, self.rolls, "tab:red", label="Roll")
        self.ax[1].plot(cT, self.pitches, "tab:green", label="Pitch")
        self.ax[2].plot(cT, self.yaws, "tab:blue", label="Yaw")
        self.ax[0].grid()
        self.ax[0].legend()
        self.ax[1].grid()
        self.ax[1].legend()
        
        self.ax[2].grid()
        self.ax[2].legend()
        
        plt.show(block="no_block" not in sys.argv)
    
    def createSystem(self):
        self.system = Sensors()
    
    def getSystem(self):
        return self.system
    
sens = Sensors()

ani = anim.FuncAnimation(sens.fig, sens.odomBasic, interval=50)

plt.show()

sens.ser.close()