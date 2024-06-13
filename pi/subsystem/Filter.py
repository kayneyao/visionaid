from typing import List, Tuple
import imufusion
import numpy
from dynarray import DynamicArray
import pi.subsystem.Sensors as Sensors

sample_rate = 100
big_number = 99999999

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
        
    def predict(self,
                IMU=List[Tuple[float]],
                GPS=List[Tuple[float]],
                Time=List[Tuple[float]]):
        self.update(IMU, GPS, Time)
        
        i = self.gyro.__len__()
        
        self.gyro[i] = self.offset.update(self.gyro[i])
        self.ahrs.update(self.gyro[i], self.acc[i], self.mag[i], self.T[i]-self.T[0])
        
        self.euler[i] = self.ahrs.quaternion.to_euler()
        
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
        
    def update(self, IMU, GPS, Time):
        self.gyro.append(IMU[0])
        self.mag.append(IMU[1])
        self.acc.append(IMU[2])
        self.GPS.append(GPS)
        self.T.append(Time)