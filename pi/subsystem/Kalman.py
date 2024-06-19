import sensors
from sensorfusion.kalmanFilter import kalmanFilter
from sensorfusion.helperMethods import helperMethods

GRAV_ACC = 9.80665
    
latLonStdDev = 2.0
altStdDev = 3.518522417151836
accEastStdDev = GRAV_ACC * 0.033436506994600976
accNorthStdDev = GRAV_ACC * 0.05355371135598354
accUpStdDev = GRAV_ACC * 0.2088683796078286
    
helperObj = helperMethods()

class kalman(object):    
    def __init__(self, initGPS, initIMU):
        self.objEast = kalmanFilter(helperObj.lonToMtrs(initGPS['lng']), \
                      initIMU["vel_east"], latLonStdDev, \
                      accEastStdDev, 0)

        self.objNorth = kalmanFilter(helperObj.latToMtrs(initGPS["lat"]), \
                      initIMU["vel_north"], latLonStdDev, \
                      accNorthStdDev, 0)
        
    def predict(self):
        self.objEast.predict(currData["abs_east_acc"] * GRAV_ACC, 
                    currData["timestamp"])
        self.objNorth.predict(currData["abs_north_acc"] * GRAV_ACC,
                    currData["timestamp"])