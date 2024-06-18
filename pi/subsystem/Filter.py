from typing import List, Tuple
import imufusion
import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib import style
import sys
import os
sys.path.append(os.path.abspath('./pi/subsystem'))
from Sensors import *
import queue

sample_rate = 100



class Filter(object):
    def __init__(self,
                 Values):
        
        self.offset = imufusion.Offset(sample_rate)
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                   0.5,  # gain
                                   2000,  # gyroscope range
                                   10,  # acceleration rejection
                                   10,  # magnetic rejection
                                   5)  # recovery trigger period = 5 seconds
        
        self.acc=numpy.empty_like([[0,0,0]])
        self.gyro=numpy.empty_like([[0,0,0]])
        self.mag=numpy.empty_like([[0,0,0]])
        self.GPS=numpy.empty_like([[0,0,0]])
        self.T=[0]
        
        self.update(Values)
        
        self.euler = queue.Queue()
        self.stats = []
        self.flags = []
        
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        plt.ion()
        plt.show()
        
        self.pitches = []
        self.yaws = []
        self.rolls = []
        
    def predict(self, debug=False, plotAngles=False, plotWindow=50):        
        i = self.gyro.__len__() - 1
        
        self.gyro[i] = self.offset.update(self.gyro[i])
        
        self.ahrs.update(self.gyro[i], self.acc[i], self.mag[i], self.T[i]-self.T[1])
        
        self.euler.put(self.ahrs.quaternion.to_euler())
        
        ahrs_internal_states = self.ahrs.internal_states
        self.stats.append(numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_recovery_trigger,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_recovery_trigger]))

        ahrs_flags = self.ahrs.flags
        self.flags.append(numpy.array([ahrs_flags.initialising,
                                ahrs_flags.angular_rate_recovery,
                                ahrs_flags.acceleration_recovery,
                                ahrs_flags.magnetic_recovery]))
        
        if(plotAngles):
            self.plot(plotWindow)
            
        if(debug):
            self.debug(plotWindow)
        
        
        
    def pose(self):
        None
        
    def plot(self, plotWindow):
        if(plotWindow < self.pitches.__len__()):
            self.pitches.pop(0)
            self.yaws.pop(0)
            self.rolls.pop(0)
        pitch, yaw, roll = self.euler.get()
        self.pitches.append(pitch)
        self.yaws.append(yaw)
        self.rolls.append(roll)
        self.ax1.clear()
        self.ax1.plot(self.rolls, "tab:red", label="Roll")
        self.ax1.plot(self.pitches, "tab:green", label="Pitch")
        self.ax1.plot(self.yaws, "tab:blue", label="Yaw")
        plt.pause(0.001)
        
    def debug(self):
        
    
    def update(self, value):
        self.gyro = np.append(self.gyro, [value[0]], axis=0)
        self.mag = np.append(self.mag, [value[1]], axis=0)
        self.acc = np.append(self.acc, [value[2]], axis=0)
        self.GPS = np.append(self.GPS, [value[3]], axis=0)
        self.T = np.append(self.T, [value[4]], axis=0)
        
Sensors.createSystem(Sensors)
        
sensors = Sensors.getSystem(Sensors)

madgwick = Filter(sensors.getValues())

while 1:
    Filter.update(madgwick, sensors.getValues())
    Filter.predict(madgwick, True, 50)