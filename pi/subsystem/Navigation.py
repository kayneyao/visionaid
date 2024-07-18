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
import matplotlib
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
        self.yaw = []
        
        self.estimate = np.empty((1, 2))

        self.start = []
        self.target = []
        
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

        with open('/home/sophie/visionaid/routes/directions.json', 'w') as f:
            json.dump(route.__dict__, f)

        return route.__dict__

    def guide(self, orient, distance):
        output = [""]
        if np.abs(orient) <= 10:
            output.append("Head straight. With ")
            output.append(np.linalg.norm(distance).astype(np.str_))
            output.append(" more meters until next waypoint.")
            return ' '.join(output)
        elif np.abs(orient) <= 50:
            output.append("Slight Turn ")
        elif np.abs(orient) <= 100 and np.abs(orient) > 50:
            output.append("Turn")
        else:
            output.append("Turn Around")
            return ' '.join(output)

        if orient < 0:
            output.append("Left")
        else:
            output.append("Right")  

        return ' '.join(output)
    
    def plot(self, gps, fpos, route, yaw):
        figure, axis = plt.subplots(2, 1) 


        axis[1].scatter(gps[:, 1], gps[:, 0])
        axis[1].plot(fpos[:, 0], fpos[:, 1])
        axis[1].plot(route[:, 0], route[:, 1])

        axis[0].plot(yaw)

        plt.show()

    def navigate(self, plot=False):
        stored = np.array(self.getDirections().get('_geometry'))

        step = 0

        start = np.append((np.array(stored[step])*self.COORD_TO_M), 20)     
        target = np.append((np.array(stored[step + 1])*self.COORD_TO_M), 20)

        self.estimate = self.sens.complimentary(start, target, initF=True)

        while np.linalg.norm(stored[-1] - self.estimate[0][:2]) > 2:

            self.start = np.append((np.array(stored[step])*self.COORD_TO_M), 20)
            try:
                self.target = np.append((np.array(stored[step + 1])*self.COORD_TO_M), 20)
            except IndexError:
                self.gps.pop(0)
                self.path.pop(0)
                self.yaw.pop(0)
                self.plot((np.array(self.gps)), np.array(self.path)/self.COORD_TO_M, stored, self.yaw)

            self.estimate = self.sens.complimentary(self.start, self.target, initF=False)
            
            step += 1

            # print(self.target)
            # print(self.estimate)
            # print(np.asin((self.target[1] - self.estimate[0][1])/(self.target[0] - self.estimate[0][0])%1)*180/np.pi)
            # print((self.estimate[1][0] - 180)*2)
            # print(self.target - self.estimate[0])
            print(self.guide(np.arcsin((self.target[1] - self.estimate[0][1])/(self.target[0] - self.estimate[0][0])%1)*180/np.pi  - (self.estimate[1][0] - 180)*2, self.target - self.estimate[0]))
            # print(np.linalg.norm(target - self.estimate[0][:2] - start))

            while np.linalg.norm(self.target - self.estimate[0]) > 0.4:
                try:
                    self.estimate = self.sens.complimentary(self.start, self.target, initF=False)

                    self.path.append(self.estimate[0][:2])
                    self.gps.append(self.sens.gps[:2])
                    self.yaw.append(2 * (self.estimate[1][0] - 180))

                except IndexError:
                    self.gps.pop(0)
                    self.path.pop(0)
                    self.yaw.pop(0)
                    # print(self.gps[0])
                    # print(self.path[0])
                    self.plot((np.array(self.gps)), np.array(self.path)/self.COORD_TO_M, stored, self.yaw)
                    # print(np.array(self.gps)[1700])
                    # print(stored[5])
                    # self.plot(np.array(self.gps), stored)

        self.plot((np.array(self.gps) + self.start/self.COORD_TO_M), np.array(self.path)/self.COORD_TO_M, stored)

        

navigate = Navigation()

navigate.navigate(True)