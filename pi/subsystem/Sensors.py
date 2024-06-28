import serial
from time import sleep
import pynmea2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import sys
import imufusion
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
    sample_rate = 100
    
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)
        sleep(2)
        
        self.output = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],0]
        
        self.pitches = []
        self.rolls = []
        self.yaws = []
        
        self.iT = self.getValues()[4]
        
        self.posX = []
        self.posY = []
        
        self.temp = []
        
        self.dx = 0
        self.dy = 0
        

        self.pitch = self.initOrien()[1]
        self.roll = self.initOrien()[2]
        self.yaw = self.initOrien()[0]
        
        self.fig , self.ax = plt.subplots(nrows=3, sharex=True,
                                          gridspec_kw={"height_ratios": [6,6,1]})    
            
    def getValues(self):
        self.ser.write(b'g')
        temp = self.ser.readline().decode().split(", ")
        data1 = np.array(temp, dtype=float)
        self.ser.write(b'g')
        temp = self.ser.readline().decode().split(", ")
        data2 = np.array(temp, dtype=float)
        
        data = (data1 + data2)/2
        
        if data.__len__() != 13:
            return self.output
        
        mag = list(map(float, data[0:3]))
        gyro = list(map(float, data[3:6]))
        acc = list(map(float, data[6:9]))
        gps = list(map(float, data[9:12]))
        time = int(data[12])
        
        # mag = self.soft @ (np.array(mag) - self.hard)
        acc = self.A @ (np.array(acc) - self.b)

        self.output = [mag, gyro, acc, gps, time]
        
        return self.output
    
    def calibrate(self, device):
        print("Rotate the Device in Different Positions")
        sleep(2)
        if device == 'accel':
            index = 2
        elif device == 'magnet':
            index = 0
            
        raw_data = []
        
        for i in range(1000):
            self.ser.write(b'g')
            temp = self.ser.readline().decode().split(", ")
            raw_data.append(list(map(float,temp[index*3: index*3+3])))
            
        raw_data = np.array(raw_data)
        
        print(raw_data[1, 0])
        
        def ellipsoid_fit(params, x, y, z):
            # Ellipsoid parameters
            center = params[:3]
            radii = params[3:6]
            rotation = params[6:9]

            # Construct rotation matrix
            Rx = np.array([[1, 0, 0],
                           [0, np.cos(rotation[0]), -np.sin(rotation[0])],
                           [0, np.sin(rotation[0]), np.cos(rotation[0])]])

            Ry = np.array([[np.cos(rotation[1]), 0, np.sin(rotation[1])],
                           [0, 1, 0],
                           [-np.sin(rotation[1]), 0, np.cos(rotation[1])]])

            Rz = np.array([[np.cos(rotation[2]), -np.sin(rotation[2]), 0],
                           [np.sin(rotation[2]), np.cos(rotation[2]), 0],
                           [0, 0, 1]])

            R = Rz @ Ry @ Rx

            # Apply transformation
            transformed_data = np.dot(R, np.array([x, y, z]) - center[:, None])
            distances = ((transformed_data[0] / radii[0]) ** 2 +
                         (transformed_data[1] / radii[1]) ** 2 +
                         (transformed_data[2] / radii[2]) ** 2 - 1)
            return distances

        # Initial guess for the parameters
        x_mean, y_mean, z_mean = np.mean(raw_data, axis=0)
        initial_params = [x_mean, y_mean, z_mean, 1, 1, 1, 0, 0, 0]

        # Optimize to find the best-fit ellipsoid parameters
        result = least_squares(ellipsoid_fit, initial_params, args=(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2]))

        # Extract the optimized parameters
        center = result.x[:3]
        radii = result.x[3:6]
        rotation = result.x[6:9]

        # Construct rotation matrix
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rotation[0]), -np.sin(rotation[0])],
                       [0, np.sin(rotation[0]), np.cos(rotation[0])]])

        Ry = np.array([[np.cos(rotation[1]), 0, np.sin(rotation[1])],
                       [0, 1, 0],
                       [-np.sin(rotation[1]), 0, np.cos(rotation[1])]])

        Rz = np.array([[np.cos(rotation[2]), -np.sin(rotation[2]), 0],
                       [np.sin(rotation[2]), np.cos(rotation[2]), 0],
                       [0, 0, 1]])

        R = Rz @ Ry @ Rx

        soft_iron_matrix = np.diag(1 / radii) @ R
        inverse_soft_iron_matrix = np.linalg.inv(soft_iron_matrix)

        return center, soft_iron_matrix, inverse_soft_iron_matrix

    def odomBasic(self, i):
        Z = self.getValues()
        
        cT = Z[4]
        dT = cT - self.iT
        self.iT = cT
        
        self.roll += (xa * 0.001 + 0.00355482) * dT
        self.pitch += (ya * 0.001 - 0.00420793) * dT
        self.yaw += (za * -0.001 + 0.00026876) * dT
        
        r = R.from_euler('zyx', [
            self.yaw,
            self.pitch,
            self.roll], degrees=True)
        
        r = R.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=True)
        r = r.inv()
        
        x = Z[2][0]
        y = Z[2][1]
        z = Z[2][2]
        [xA, yA, zA] = r.apply([x, y, z - 9.80655]) 
        # [xA, yA, zA] = self.ahrs.linear_acceleration - r.apply([0,0,9.80655])
        
        print(r.as_euler('xyz', degrees=True))
        
        self.dx += xA * (dT/1000)**2
        self.dy += yA * (dT/1000)**2
        
        self.posX.append(self.dx)
        self.posY.append(self.dy)
        
        self.temp.append(zA)
        
        self.ax[0].clear()
        self.ax[1].clear()
        self.ax[0].plot(self.posX, self.posY)
        self.ax[1].plot(self.temp[-10:])

        self.ax[0].set_title("Orientation")
        self.ax[0].set_ylabel("Y-Axis")
        self.ax[0].set_xlabel("X-Axis")
    
    def initOrien(self):
        acc = []
        mag = []
        
        for i in range(4):
            Z = self.getValues()
            acc.append(Z[2])
            mag.append(Z[0])
        
        accMean = np.mean(acc, 0)
        magMean = np.mean(mag, 0)
        
        magNorm = magMean / np.linalg.norm(magMean)
        accNorm = accMean / np.linalg.norm(accMean)
        
        D = -accNorm
        E = np.cross(D, magNorm)
        E = E / np.linalg.norm(E)
        N = np.cross(E,D)
        N = N / np.linalg.norm(N)
        
        return R.from_matrix([[N[0], E[0], D[0]],
                              [N[1], E[1], D[1]],
                              [N[2], E[2], D[2]],]).as_euler('zyx', degrees=True)
        
    def gain(self, ratio, arr1, arr2):
        arr1 = np.array(arr1)
        arr2 = np.array(arr2)
        return (arr1*ratio[0] + arr2*ratio[1])/(ratio[0] + ratio[1])
    
    def gyroOrien(self):
        # Z = np.genfromtxt("./pi/subsystem/sensor.csv", delimiter=",", skip_header=1)
        Z = self.getValues()
        # print(Z)
        
        mag = Z[0]
        magNorm = mag / np.linalg.norm(mag)
        acc = Z[2]
        accNorm = acc / np.linalg.norm(acc)
        
        D = accNorm
        E = np.cross(D, magNorm)
        E = E / np.linalg.norm(E)
        N = np.cross(E, D)
        N = N / np.linalg.norm(N)
        
        accMagOrien = R.from_matrix([[N[0], E[0], D[0]],
                           [N[1], E[1], D[1]],
                           [N[2], E[2], D[2]],]).as_euler('zyx', degrees=True)
        
        cT = Z[4]
        dT = cT - self.iT
        self.iT = cT
        
        xg = Z[1][0]
        yg = Z[1][1]
        zg = Z[1][2]
        
        self.roll += (xg * 0.001 + 0.00355482) * dT
        self.pitch += (yg * -0.001 + 0.00420793) * dT
        self.yaw += (zg * 0.001 - 0.00026876) * dT
        
        # [y,p,r] = self.gain([1, 5],
        #                     [self.yaw, self.pitch, self.roll],
        #                     accMagOrien)
        [self.yaw,self.pitch,self.roll] = self.gain([49, 1],
                            [self.yaw, self.pitch, self.roll],
                            accMagOrien)
        print(accMagOrien)
        # return [self.yaw,self.pitch,self.roll]
        return accMagOrien
    

# ani = anim.FuncAnimation(sens.fig, sens.odomBasic, interval=50)

# plt.show()

# sens.ser.close()