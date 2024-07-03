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
    
    # soft = np.array([[1.416250, 0.058590, 0.135729],
    #             [0.058590, 1.496296, 0.196961],
    #             [0.135729, 0.196961, 1.258746]])
    # hard = np.array([-14.852945, -54.355349, -65.776942])
    # soft = np.array([[4.37818397e-07, -2.92218222e-07,  6.14973875e-08],
    #    [-1.73511405e-05, -2.75330302e-06,  1.10445069e-04],
    #    [-3.71788902e-05, -5.72330620e-05, -7.26765003e-06]])
    # hard = np.array([-14.852945, -54.355349, -65.776942])
    soft = np.array([[1.924639, -0.001190, -0.037292],
                     [-0.001190, 1.902003, 0.081436],
                     [-0.037292, 0.081436, 2.046809]])
    hard = np.array([-3.346703, 30.085820, -10.517505])
        
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
        

        self.pitch = -self.initOrien()[1]
        self.roll = -self.initOrien()[2]
        self.yaw = self.initOrien()[0]
        
        self.fig , self.ax = plt.subplots(nrows=3, sharex=True,
                                          gridspec_kw={"height_ratios": [6,6,1]})    
            
    def getValues(self):
        self.ser.write(b'g')
        temp = self.ser.readline().decode().split(", ")
        if temp.__len__() != 13:
            return self.output
        data1 = np.array(temp, dtype=float)
        self.ser.write(b'g')
        temp = self.ser.readline().decode().split(", ")
        if temp.__len__() != 13:
            return self.output
        data2 = np.array(temp, dtype=float)
        
        data = (data1 + data2)/2     
        
        # print(data)
        
        mag = list(map(float, data[0:3]))
        gyro = list(map(float, data[3:6]))
        acc = list(map(float, data[6:9]))
        gps = list(map(float, data[9:12]))
        time = int(data[12])
        
        mag = self.soft @ (np.array(mag) - self.hard)
        # acc = self.A @ (np.array(acc) - self.b)

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
            sleep(0.01)
            
        raw_data = np.array(raw_data)
        
        def ellipsoid_fit(points):
            # Ellipsoid fit equation: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
            def residuals(params, x, y, z):
                A, B, C, D, E, F, G, H, I, J = params
                return A*x**2 + B*y**2 + C*z**2 + 2*D*x*y + 2*E*x*z + 2*F*y*z + 2*G*x + 2*H*y + 2*I*z + J

            x, y, z = points.T
            initial_params = np.zeros(10)
            result = least_squares(residuals, initial_params, args=(x, y, z))
            return result.x
        
        params = ellipsoid_fit(raw_data)
        A, B, C, D, E, F, G, H, I, J = params

        # Hard iron correction (offsets)
        offset_x = -G/A
        offset_y = -H/B
        offset_z = -I/C

        # Soft iron correction (scaling and rotation)
        A, B, C = np.abs([A, B, C])  # Ensure positive diagonal values
        scale_factors = np.sqrt([A, B, C])
        D, E, F = D/scale_factors[0], E/scale_factors[0], F/scale_factors[0]
        correction_matrix = np.array([[A, D, E],
                                    [D, B, F],
                                    [E, F, C]])
        correction_matrix /= scale_factors

        return np.array([offset_x, offset_y, offset_z]), np.linalg.inv(correction_matrix)

        
        

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
        diff = ( ( arr1 - arr2 + 180 + 360 ) % 360 ) - 180
        ypr = (360 + arr2 + (ratio[0] * diff / (ratio[0] + ratio[1])) ) % 360
        return ypr
    
    def gyroOrien(self):
        # Z = np.genfromtxt("./pi/subsystem/sensor.csv", delimiter=",", skip_header=1)
        Z = self.getValues()
        # print(Z)
        
        mag = Z[0]
        magNorm = mag / -np.linalg.norm(mag)
        acc = Z[2]
        accNorm = acc / -np.linalg.norm(acc)
        
        D = -accNorm
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
        
        self.roll += (xg * 0.001) * dT
        self.pitch += (yg * 0.001) * dT
        self.yaw += (zg * 0.001) * dT
        
        # [y,p,r] = self.gain([1, 5],
        #                     [self.yaw, self.pitch, self.roll],
        #                     accMagOrien)
        [self.yaw,self.pitch,self.roll] = self.gain([49, 1],
                            [self.yaw, self.pitch, self.roll],
                            [accMagOrien[0], accMagOrien[1], accMagOrien[2]])
        print(accMagOrien[0])
        print(self.yaw)
        return [self.yaw,self.pitch,self.roll]
        # return accMagOrien
    
# sens = Sensors()

# print(sens.calibrate('magnet'))

# ani = anim.FuncAnimation(sens.fig, sens.odomBasic, interval=50)

# plt.show()

# sens.ser.close()