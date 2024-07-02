from filterpy.kalman import KalmanFilter
from filterpy.gh import GHFilter
import numpy as np
import matplotlib.pyplot as plt

class Filter(object):
    def __init__(self, ):
        pass
    
    def kalman(self):
        pass
    def gh(self, z, g, h):
        self.ghFilter.update(z=z, g=g, h=h)
    def avg(self):
        pass
    
    def plot(self, plotWindow):
        if(plotWindow < self.pitches.__len__()):
            self.pitches.pop(0)
            self.yaws.pop(0)
            self.rolls.pop(0)
        pitch, yaw, roll = self.euler.get()
        self.pitches.append(pitch)
        self.yaws.append(yaw)
        self.rolls.append(roll)
        self.ax[0].clear()
        self.ax[1].clear()
        
        self.ax[2].clear()
        self.ax[0].plot(self.pitches, "tab:red", label="Pitch")
        self.ax[1].plot(self.rolls, "tab:green", label="Roll")
        self.ax[2].plot(self.yaws, "tab:blue", label="Yaw")
        self.ax[0].grid()
        self.ax[0].legend()
        self.ax[1].grid()
        self.ax[1].legend()
        
        self.ax[2].grid()
        self.ax[2].legend()