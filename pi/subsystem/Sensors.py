import serial
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import expm, block_diag
import json
# import RPi.GPIO as GPIO

class Sensors(object):
    COORD_TO_M = 1852

    t = 0 
    
    soft = np.array([[1.686350, 0.061805, 0.053021],
                [0.061805, 1.649245, 0.123506],
                [0.053021, 0.123506, 1.781350]])
    hard = np.array([-39.739560, 65.354347, -30.261158])
        
    A = np.array([[0.999265, -0.013986, 0.000582],  # 'A^-1' matrix from Magneto
              [-0.013986, 0.997537, -0.001473],
              [0.000582, -0.001473, 0.989939]])
# 'Combined bias (b)' vector from Magneto
    b = np.array([0.047527, 0.021009, -0.071754])
    
    def __init__(self):
        self.ser = serial.Serial('COM3', 9600, timeout=5.0)
        sleep(2)
        
        self.output = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],0]
        
        self.pitches = []
        self.rolls = []
        self.yaws = []
        
        
        
        self.posX = []
        self.posY = []
        
        self.temp = []
        
        self.dx, self.dy, self.dz, self.vx, self.vy, self.vz = 0, 0, 0, 0, 0, 0

        

        self.compInit = [R.from_euler('Z', 0, degrees=True), 0, 0, 0, np.empty((1, 2))]

        self.saved_data = open('./data/serial_data2.txt', 'r').readlines()

        # self.saved_data.pop(0)        
        # print(self.saved_data[0].split(", "))

        self.iT = self.getValues()[4]

        self.pitch = -self.initOrien()[1]
        self.roll = -self.initOrien()[2]
        self.yaw = self.initOrien()[0]
        # print(self.saved_data)
        
        # self.fig , self.ax = plt.subplots(nrows=3, sharex=True,
        #                                   gridspec_kw={"height_ratios": [6,6,1]})    
            
    def getValues(self):
        # self.ser.write(b'g')
        # temp = self.ser.readline().decode().split(", ")
        # if temp.__len__() != 13:
        #     return self.output
        # data1 = np.array(temp, dtype=float)

        # self.ser.write(b'g')
        # temp = self.ser.readline().decode().split(", ")
        # if temp.__len__() != 13:
        #     return self.output
        # data2 = np.array(temp, dtype=float)

        

        temp = self.saved_data[0].split(", ")
        if temp.__len__() != 13:
            return self.output
        self.saved_data.pop(0)
        data1 = np.array(temp, dtype=float)

        temp = self.saved_data[0].split(", ")
        if temp.__len__() != 13:
            return self.output
        self.saved_data.pop(0)
        data2 = np.array(temp, dtype=float)
        
        data = (data1 + data2)/2
        
        mag = list(map(float, data[0:3]))
        gyro = list(map(float, data[3:6]))
        acc = list(map(float, data[6:9]))
        [lat, lon, alt] = list(map(float, data[9:12]))
        time = int(data[12])
        
        mag = self.soft @ (np.array(mag) - self.hard)
        acc = self.A @ (np.array(acc) - self.b)

        acc *= 9.806526860225

        gps = [lat/5000000 - 25.0202, lon/10000000, alt/1000]

        self.output = [mag, gyro, acc, gps, time]
        
        return self.output
    
    # def getData(self):
    #     self.ser.write(b'g')
    #     temp = .split(", ")
    #     if temp.__len__() != 13:
    #         return self.output
    #     data1 = np.array(temp, dtype=float)

    #     self.ser.write(b'g')
    #     temp = self.ser.readline().decode().split(", ")
    #     if temp.__len__() != 13:
    #         return self.output
    #     data2 = np.array(temp, dtype=float)
        
    #     data = (data1 + data2)/2
        
    #     mag = list(map(float, data[0:3]))
    #     gyro = list(map(float, data[3:6]))
    #     acc = list(map(float, data[6:9]))
    #     gps = list(map(float, data[9:12]))
    #     time = int(data[12])
        
    #     mag = self.soft @ (np.array(mag) - self.hard)
    #     acc = self.A @ (np.array(acc) - self.b)



    #     self.output = [mag, gyro, acc, gps, time]
        
    #     return self.output

    def complimentary(self, start, end, initF=False):
        def init():
            Z = self.getValues()
            #Use North Direction, Starting Point, and Target Point to Find Trajectory
            target = (start - end)
            [self.dx, self.dy, self.dz] = start
            north = self.getNorth(Z)
            #Find Initial GPS Values to Use as Origin
            ilat, ilng, ialt = Z[3]

            #Returns Trajectory from Starting Point to Target
            return [R.from_euler('z',
                        np.dot(north, target)/(np.linalg.norm(north)*np.linalg.norm(north)),
                        degrees=False), ilat, ilng, ialt, target]

        def update(gain, ilat, ilng, ialt):
            self.gps = self.getValues()[3]
            clat, clng, calt = self.gps
            #Deg to Rad
            # clat *= np.pi/180
            # clng *= np.pi/180
            # ilat *= np.pi/180
            # ilng *= np.pi/180
            # #Calculate Displacement
            # lat = clat - ilat
            # lng = clng - ilng
            #Represent as XYZ Coordinates
            x = clng * self.COORD_TO_M
            y = clat * self.COORD_TO_M
            z = 20
            #Complimentary Filter
            [self.dx, self.dy, self.dz] = (np.array([x, y, z])*gain + np.array([self.dx,self.dy,self.dz]))/(gain+1)
            return [self.dx, self.dy, self.dz]
        
        #Get Initial Values
        if(initF == True):
            self.compInit = init()
            print(self.compInit)
        #Transform to Face Trajectory
        Z = self.getValues()
        #Predict Step
        self.odomBasic(Z)
        #Update Step
        update(1, self.compInit[1], self.compInit[2], self.compInit[3])
        return np.array([[self.dx, self.dy, self.dz], [self.yaw, self.pitch, self.roll]])
    
    def kalman(self, start, end, phi=0.1): #Tracking position and velocity
        ilat, ilng, ialt = self.getValues()[3]

        dT = self.dT
        filter = KalmanFilter(dim_x=9, dim_z=3)
        filter.F = np.array([[1, 0, 0, dT, 0, 0, d2T, 0,   0],
                                [0, 1, 0, 0,  dT,0, 0,   d2T, 0],
                                [0, 0, 1, 0,  0, dT,0,   0,   d2T],
                                [0, 0, 0, 1,  0, 0, dT,  0,   0],
                                [0, 0, 0, 0,  1, 0, 0,   dT,  0],
                                [0, 0, 0, 0,  0, 1, 0,   0,   dT],
                                [0, 0, 0, 0,  0, 0, 1,   0,   0],
                                [0, 0, 0, 0,  0, 0, 0,   1,   0],
                                [0, 0, 0, 0,  0, 0, 0,   0,   1]])
        
        q = Q_discrete_white_noise(dim=3, dt=dT, var=phi)
        filter.R = np.array([5, 0, 0],
                            [0, 5, 0],
                            [0, 0, 5]).T
        
        filter.P = np.eye(9)*100

        filter.Q = block_diag(q, q, q)

        filter.H = np.array([[d2T, 0,   0,   dT, 0, 0, 1, 0, 0], #x
                             [0,   d2T, 0,   0, dT, 0, 0, 1, 0], #y
                             [0,   0,   d2T, 0, 0, dT, 0, 0, 1]])#z
        
        filter.x = [[0, 0, 0, 0]]
        
        def update(ilat, ilng, R):

            clat, clng, calt = self.getValues()[3]
            # Convert degrees to radians
            clat *= np.pi/180
            clng *= np.pi/180

            ilat *= np.pi/180
            ilng *= np.pi/180
            
            # Calculate differences
            lat = ilat - clat
            lng = ilng - clng
            
            # Calculate x and y
            x = lng * R * np.cos(ilat)
            y = lat * R

            kf.update(x, y)
        
        kf = init()

        kf.predict()
        kf.update(self.odomBasic())


    def odomBasic(self, Z):
        #Calculate Delta Time
        cT = Z[4]
        self.dT = cT - self.iT
        self.iT = cT
        #Updates Orientation inside Sensors object
        self.gyroOrien(Z) 
        #Get Inverse of System Orientation
        r = R.from_euler('zyx', [self.yaw,-self.pitch,-self.roll], degrees=True).inv() 
        #Assign Variables to Acceleromter Readings
        x = Z[2][0]
        y = Z[2][1]
        z = Z[2][2]
        #Transform System Acceleration to World Frame and Remove Gravity
        [xA, yA, zA] = r.apply([x, y, z])
        zA -= 9.806526860225
        #Integrate State Variables
        self.dx += xA * (self.dT/1000)**2
        self.dy += yA * (self.dT/1000)**2
        self.dz += zA * (self.dT/1000)**2
        self.vx += xA * (self.dT/1000)
        self.vy += yA * (self.dT/1000)
        self.vz += zA * (self.dT/1000)
        #Return Absolute Acceleration
        return [xA, yA, zA]
    
    def initOrien(self):
        acc = []
        mag = []
        
        for i in range(4):
            Z = self.getValues()
            acc.append(Z[2])
            mag.append(Z[0])
        
        accMean = np.mean(acc, 0)
        magMean = np.mean(mag, 0)
        
        magNorm = magMean / -np.linalg.norm(magMean)
        accNorm = accMean / -np.linalg.norm(accMean)
        
        D = -accNorm
        E = np.cross(D, magNorm)
        E = E / np.linalg.norm(E)
        N = np.cross(E,D)
        N = N / np.linalg.norm(N)
        
        return R.from_matrix([[N[0], E[0], D[0]],
                              [N[1], E[1], D[1]],
                              [N[2], E[2], D[2]],]).as_euler('zyx', degrees=True)
    
    def getNorth(self, Z):
        mag = Z[0]
        magNorm = mag / -np.linalg.norm(mag)
        acc = Z[2]
        accNorm = acc / -np.linalg.norm(acc)
        
        D = -accNorm
        E = np.cross(D, magNorm)
        E = E / np.linalg.norm(E)
        N = np.cross(E, D)
        return np.cross(E,D) / np.linalg.norm(N)

        
    def gainAngle(self, ratio, arr1, arr2):
        arr1 = np.array(arr1) #Declare Arrays as Numpy Arrays
        arr2 = np.array(arr2)
        diff = ((arr1 - arr2 + 180 + 360) % 360) - 180 #Calculate Difference on Circular Range
        ypr = (360 + arr2 + (ratio[0] * diff / (ratio[0] + ratio[1]))) % 360 #Average
        return ypr
    
    def gyroOrien(self, Z):
        # Z = self.getValues()
        #Obtain Unit Vectors of Mag and Accel Measurements
        mag = Z[0] 
        magNorm = mag / -np.linalg.norm(mag)
        acc = Z[2]
        accNorm = acc / -np.linalg.norm(acc)
        #Caluculate Direction Cosine Matrix (DCM) with Cross Products
        D = -accNorm #Acceleration will Oppose Gravity which is Down
        E = np.cross(D, magNorm) #East is Perpendicular to Mag and Down
        E = E / np.linalg.norm(E)
        N = np.cross(E, D) #North is Perpendicular to East and Down
        N = N / np.linalg.norm(N)
        #Create Rotation Object to Transform DCM to Euler Angles
        accMagOrien = R.from_matrix([[N[0], E[0], D[0]], 
                           [N[1], E[1], D[1]],
                           [N[2], E[2], D[2]],]).as_euler('zyx', degrees=True)
        
        #Get Change in Time
        cT = Z[4]
        dT = cT - self.iT
        self.iT = cT
        #Retreive Gyro Values
        xg = Z[1][0]
        yg = Z[1][1]
        zg = Z[1][2]
        #Integrate Gyro Values (Time is Measured in ms, Gyro Outputs deg/s)
        self.roll += (xg * 0.001) * dT
        self.pitch += (yg * 0.001) * dT
        self.yaw += (zg * 0.001) * dT
        #Combine Sensor Values
        [self.yaw,self.pitch,self.roll] = self.gainAngle([0, 1],
                            [self.yaw, self.pitch, self.roll],
                            [accMagOrien[0], accMagOrien[1], accMagOrien[2]])
        return [self.yaw,-self.pitch,-self.roll]
        # return accMagOrien
    
# sens = Sensors()

# # print(sens.calibrate('magnet'))

# ani = anim.FuncAnimation(sens.fig, sens.odomBasic, interval=50)

# plt.show()

# sens.ser.close()