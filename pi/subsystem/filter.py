from typing import List, Tuple
import imufusion
import sensors

sample_rate = 100

class filter(object):
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
        
    def predict(self,
                IMU=List[Tuple[float]],
                GPS=List[Tuple[float]],
                Time=List[Tuple[float]]):
        self.gyro.append(IMU[0])
        self.mag.append(IMU[1])
        self.acc.append(IMU[2])
        self.GPS.append(GPS)
        self.T.append(Time)
        i = self.gyro.__len__()
        
        euler = [[()]]
        
        self.gyro[i] = self.offset.update(self.gyro[i])
        self.ahrs.update(self.gyro[i], self.acc[i], self.mag[i], self.T[i]-self.T[0])
        
        euler[i] = self.ahrs.quaternion.to_euler()
        
        absAcc = euler[i]
        
        position = 
        
        return position