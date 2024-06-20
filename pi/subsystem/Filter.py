import imufusion
import numpy
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
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
                                   5*sample_rate)  # recovery trigger period = 5 seconds
        
        self.acc=numpy.empty_like([[0,0,0]])
        self.gyro=numpy.empty_like([[0,0,0]])
        self.mag=numpy.empty_like([[0,0,0]])
        self.GPS=numpy.empty_like([[0,0,0]])
        self.T=[0]
        
        self.update(Values)
        
        self.euler = queue.Queue()
        self.states = queue.Queue()
        self.flags = queue.Queue()
        
        self.fig , self.ax = plt.subplots(nrows=9, sharex=True, gridspec_kw={"height_ratios": [3,3,3,1,1,1,1,1,1]})
        plt.ion()
        plt.show()
        
        self.pitches = []
        self.yaws = []
        self.rolls = []
        
        self.accErs = []
        self.accIgs = []
        self.accRes = []
        self.magErs = []
        self.magIgs = []
        self.magRes = []
        
        self.i = 2
        
        self.filterP = []
        
    def predict(self, debug=False, plotAngles=False, plotWindow=50):        
        
        i = self.gyro.__len__() - 1
        
        self.gyro[i] = self.offset.update(self.gyro[i])
        
        self.ahrs.update(self.gyro[i], self.acc[i], self.mag[i], self.T[i]/1000)
        
        self.euler.put(self.ahrs.quaternion.to_euler())
        
        ahrs_internal_states = self.ahrs.internal_states
        self.states.put([ahrs_internal_states.acceleration_error,
                        ahrs_internal_states.accelerometer_ignored,
                        ahrs_internal_states.acceleration_recovery_trigger,
                        ahrs_internal_states.magnetic_error,
                        ahrs_internal_states.magnetometer_ignored,
                        ahrs_internal_states.magnetic_recovery_trigger])

        ahrs_flags = self.ahrs.flags
        self.flags.put([ahrs_flags.initialising,
                        ahrs_flags.angular_rate_recovery,
                        ahrs_flags.acceleration_recovery,
                        ahrs_flags.magnetic_recovery])
        
        if(plotAngles):
            self.plot(plotWindow)
            
        if(debug):
            self.debug(plotWindow)
        plt.pause(0.001)
        
        
        
        
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
        self.filterP = uniform_filter1d(self.pitches, self.i)[1:]
        self.ax[0].clear()
        self.ax[1].clear()
        
        # self.ax[2].clear()
        self.ax[0].plot(self.pitches, "tab:red", label="Pitch")
        self.ax[1].plot(self.filterP, "tab:green", label="Filter")
        # print(self.filterP)
        # self.ax[2].plot(self.yaws, "tab:blue", label="Yaw")
        self.ax[0].grid()
        self.ax[0].legend()
        self.ax[1].grid()
        self.ax[1].legend()
        
        # self.ax[2].grid()
        # self.ax[2].legend()
        
    def debug(self, plotWindow):
        if(plotWindow < self.accErs.__len__()):
            self.accErs.pop(0)
            self.accIgs.pop(0)
            self.accRes.pop(0)
            self.magErs.pop(0)
            self.magIgs.pop(0)
            self.magRes.pop(0)
        accEr, accIg, accRe, magEr, magIg, magRe = self.states.get()
        self.accErs.append(accEr)
        self.accIgs.append(accIg)
        self.accRes.append(accRe)
        self.magErs.append(magEr)
        self.magIgs.append(magIg)
        self.magRes.append(magRe)
        self.ax[3].clear()
        self.ax[4].clear()
        self.ax[5].clear()
        self.ax[6].clear()
        self.ax[7].clear()
        self.ax[8].clear()
        self.ax[3].plot(self.accErs, label='accErs')
        self.ax[4].plot(self.accIgs, label='accIgs')
        self.ax[5].plot(self.accRes, label='accRes')
        self.ax[6].plot(self.magErs, label='magErs')
        self.ax[7].plot(self.magIgs, label='magIgs')
        self.ax[8].plot(self.magRes, label='magRes')
        self.ax[3].grid()
        self.ax[3].legend()
        self.ax[4].grid()
        self.ax[4].legend()
        self.ax[5].grid()
        self.ax[5].legend()
        self.ax[6].grid()
        self.ax[6].legend()
        self.ax[7].grid()
        self.ax[7].legend()
        self.ax[8].grid()
        self.ax[8].legend()
        
    
    def update(self, value):
        self.gyro = np.append(self.gyro, [value[0]], axis=0)
        self.mag = np.append(self.mag, [value[1]], axis=0)
        self.acc = np.append(self.acc, [value[2]], axis=0)
        self.GPS = np.append(self.GPS, [value[3]], axis=0)
        self.T = np.append(self.T, [value[4]], axis=0)
        #blue high, green average, red, high
        


Sensors.createSystem(Sensors)
        
sensors = Sensors.getSystem(Sensors)

madgwick = Filter(sensors.getValues())

while 1:
    Filter.update(madgwick, sensors.getValues())
    Filter.predict(madgwick, False, True, 100)