from typing import List, Tuple
import imufusion
import numpy
from dynarray import DynamicArray
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib import style
import sys
sys.path.insert(1,'./pi/subsystem')
from subsystem import Sensors
import json

sample_rate = 100

class Filter(object):
    def __init__(self,
                 IMU=List[Tuple[float]],
                 GPS=List[Tuple[float]],
                 Time=List[Tuple[float]]):
        
        self.offset = imufusion.Offset(sample_rate)
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                   0.5,  # gain
                                   2000,  # gyroscope range
                                   10,  # acceleration rejection
                                   10,  # magnetic rejection
                                   5)  # recovery trigger period = 5 seconds
        
        self.gyro.append(IMU[0])
        self.mag.append(IMU[1])
        self.acc.append(IMU[2])
        self.GPS.append(GPS)
        self.T.append(Time)
        
        self.euler = DynamicArray((None, 3))
        self.stats = DynamicArray((None, 6))
        self.flags = DynamicArray((None, 4))
        style.use('fivethrityeight')
        
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1, 1)
        
    def predict(self):
        self.update(self.IMU, [0,0,0], self.Time)
        
        i = self.gyro.__len__()
        
        self.gyro[i] = self.offset.update(self.gyro[i])
        self.ahrs.update(self.gyro[i], self.acc[i], self.mag[i], self.T[i]-self.T[0])
        
        self.euler[i] = self.ahrs.quaternion.to_euler()
        
        with open('./data/madgwick.json', 'w') as f:
            json.dump(self.euler[i], f)
        
        ahrs_internal_states = self.ahrs.internal_states
        self.stats[i] = numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_recovery_trigger,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_recovery_trigger])

        ahrs_flags = self.ahrs.flags
        self.flags[i] = numpy.array([ahrs_flags.initialising,
                                ahrs_flags.angular_rate_recovery,
                                ahrs_flags.acceleration_recovery,
                                ahrs_flags.magnetic_recovery])
        
        
        
    def pose(self):
        None
        
    def plot(self, i):
        graph_data = open()
        lines = graph_data.split('\n')
        times = []
        pitches = []
        yaws = []
        rolls = []
        for line in lines:
            if len(line) > 1:
                pitch, yaw, roll = line.split(',')
                pitches.append(pitch)
                yaws.append(yaw)
                rolls.append(roll)
        self.ax1.clear()
        self.ax1.plot(pitch, yaw, roll)
    
    def update(self, IMU, GPS, Time):
        self.gyro.append(IMU[0])
        self.mag.append(IMU[1])
        self.acc.append(IMU[2])
        self.GPS.append(GPS)
        self.T.append(Time)
        
sensors = Sensors.getSystem()

