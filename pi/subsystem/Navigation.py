from typing import List, Tuple
from pyttsx3 import say
import json
from scipy import Rotation as R
from geopy import distance as d
import Sensors
from filterpy.kalman import KalmanFilter
import numpy as np
from numpy.linalg import norm
import asyncio


class Navigation(object):
    COORD_TO_M = 1852

    def __init__(self):
        self.sens = Sensors.Sensors()
        self.filter = KalmanFilter(dim_x=3, dim_z=2)

        
#transcribe directions as english
    def transDir(self):
        stored_route = json.load(open('./routes/directions.json'))
        instruction = stored_route.get('routes')[0].get('segments')[0].get('steps')

        waypoint = ['']

        for parse in instruction:
            output = str(parse.get('instruction'))
            if str(parse.get('instruction'))[:6] == 'Arrive':
                waypoint.append[output]
            elif str(parse.get('instruction'))[:4] != 'Turn':
                waypoint.append(output + ' for ' + str(parse.get('distance')) + ' meters')
            else:
                waypoint.append(output)
                waypoint.append('And head straight for ' + str(parse.get('distance')) +' meters')

        return waypoint

    def coordDist(self, coord1, coord2):
        coord1: Tuple[float, float]
        coord2: Tuple[float, float]

        coord1 = (coord1[1], coord1[0])
        coord2 = (coord2[1], coord2[0])

        dist = d.distance(coord1, coord2).m
        return dist

    def complimentary(self, start, end):
        target = (self.iPos - self.tPos)*self.COORD_TO_M
        north = self.sens.gyroOrien()[:, 1]

        transform = R.from_euler(np.acos('z',
                    np.dot(north, target)/(norm(north)*norm(north)),
                    degrees=False))
        p = self.sens.odomBasic()[0]
        v = self.sens.odomBasic()[1]

        def update(gain):
            self.sens.getValues()[3]

        return np.array([p, v])