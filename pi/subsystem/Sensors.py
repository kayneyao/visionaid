import serial
import pynmea2
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
        
        # GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(26, GPIO.OUT)

        # GPIO.output(26, GPIO.HIGH)
    
    def getGPS(self):
        coord = [0,0,0]
        while 1:
            newdata = self.ser.readline()
            n_data = newdata.decode('latin-1')
            if n_data[:6] == '$GNGGA':
                newmsg = pynmea2.parse(n_data)
                lat = newmsg.latitude
                lng = newmsg.longitude
                alt = newmsg.altitude
                coord = [lng,lat,alt]
                # print(gps)
                return coord
            
    def getIMU(self):
        data = self.ser.readline().decode().split(", ")
        
        mag = list(map(float, data[0:3]))
        gyro = list(map(float, data[3:6]))
        acc = list(map(float, data[6:9]))
        
        mag = self.soft @ (np.array(mag) - self.hard)
        
        acc = self.A @ (np.array(acc) - self.b)
        
        imu = [mag, gyro, acc]
        return imu
        
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
    
    def createSystem(self):
        self.system = Sensors()
    
    def getSystem(self):
        return self.system