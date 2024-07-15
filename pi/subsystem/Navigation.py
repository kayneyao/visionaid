import routingpy as rp
from typing import List, Tuple
# from pyttsx3 import say
import json
from scipy.spatial.transform import Rotation as R
from geopy import distance as d
from geopy.geocoders import Nominatim
import Sensors
# from filterpy.kalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
# from numpy.linalg import norm
# from enum import Enum
# import asyncio


class Navigation(object):
    COORD_TO_M = 1852

    def __init__(self):
        self.sens = Sensors.Sensors()
        key_ors = '5b3ce3597851110001cf62483c18fce726724c5a8705062fdb07ad7e'
        self.ors = rp.ORS(api_key=key_ors)

        self.nom = Nominatim(user_agent='Geopy Library')

        self.gps = [[]]
        self.path = [[]]
        
        self.estimate = np.empty((1, 2))

        self.start = [[]]
        self.target = [[]]
        
#transcribe directions as english
    def transDir(self, stored):
        print(stored)
        self.instruction = stored.get('routes')[0].get('segments')[0].get('steps')

        self.waypoint = ['']

        for parse in self.instruction:
            output = str(parse.get('instruction'))
            if str(parse.get('instruction'))[:6] == 'Arrive':
                self.waypoint.append[output]
            elif str(parse.get('instruction'))[:4] != 'Turn':
                self.waypoint.append(output + ' for ' + str(parse.get('distance')) + ' meters')
            else:
                self.waypoint.append(output)
                self.waypoint.append('And head straight for ' + str(parse.get('distance')) +' meters')

        return self.waypoint

    def coordDist(self, coord1, coord2):
        coord1: Tuple[float, float]
        coord2: Tuple[float, float]

        coord1 = (coord1[1], coord1[0])
        coord2 = (coord2[1], coord2[0])

        dist = d.distance(coord1, coord2).m
        return dist

    def getDestination(self):
        print('Please input the address')
        address = input()
        location = self.nom.geocode(address)

        print(location)

        while location == None:
            print("Please try again with a different input")
            address = input()
            location = self.nom.geocode(address)
        
        destination = [location.longitude, location.latitude]

        print(destination)

        return destination
        

    def getDirections(self):
        # coords = [self.getDestination(), self.sens.getValues()[3][0:2]]
        coords = [self.getDestination(), self.getDestination()]

        route = self.ors.directions(
            locations=coords,
            profile='foot-walking',
            format='json', 
            units='m',
            language='en',
            instructions=True,
            instructions_format='text')

        with open('./routes/directions.json', 'w') as f:
            json.dump(route.__dict__, f)

        return route.__dict__

    def guide(self, orient, distance):
        output = [""]
        if np.abs(orient) <= 5:
            output.append("Head straight. With ")
            output.append(distance)
            output.append(" more meters until next waypoint.")
            return output
        
        if np.abs(orient) <= 50:
            output.append("Slight Turn ")
        elif np.abs(orient) <= 100 and np.abs(orient) > 50:
            output.append("Turn")

        if orient < 0:
            output.append("Left")
        else:
            output.append("Right")

        return output
    
    def plot(self, gps, fpos, route):
        plt.scatter(gps[:, 1], gps[:, 0])
        plt.plot(fpos[:, 0], fpos[:, 1])
        plt.plot(route[:, 0], route[:, 1])
        plt.show()

    def navigate(self, plot=False):
        stored = np.array(self.getDirections().get('_geometry'))

        print(stored[-1])

        

        step = 1

        start = np.array(stored[step])*self.COORD_TO_M
        target = np.array(stored[step + 1])*self.COORD_TO_M

        self.estimate = self.sens.complimentary(start, target, init=True)

        while np.linalg.norm(stored[-1] - self.estimate[0][:2]) > 2:

            self.start = np.array(stored[step])*self.COORD_TO_M
            self.target = np.array(stored[step + 1])*self.COORD_TO_M

            self.estimate = self.sens.complimentary(start*self.COORD_TO_M, target*self.COORD_TO_M, init=True)
            
            step += 1

            print(start-target)
            # print(np.linalg.norm(target - self.estimate[0][:2] - start))

            while np.linalg.norm(target - self.estimate[0][:2] - start) > 0.5:
                self.estimate = self.sens.complimentary(start, target)
                self.guide(self.sens.gyroOrien()[0], target - self.estimate[0][:2])

                self.path.append(self.estimate[0][:2])
                self.gps.append(self.sens.gps)

        self.plot((np.array(self.gps) + self.start), self.path, np.array(self.stored.get("_geometry")))

        

navigate = Navigation()

navigate.navigate(True)