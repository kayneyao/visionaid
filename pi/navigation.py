from typing import List, Tuple
from pyttsx3 import say
import json
from geopy import distance as d
import sys
sys.path.insert(1,'/home/kanye/GY-85_Raspberry-Pi/i2clibraries')
import numpy as np
import asyncio


#transcribe directions as english
def transDir():
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
            waypoint.append('And head straight for ' + str(parse.get('distance')) + ' meters')

    return waypoint

def coordDist(coord1, coord2):
    coord1: Tuple[float, float]
    coord2: Tuple[float, float]

    coord1 = (coord1[1], coord1[0])
    coord2 = (coord2[1], coord2[0])

    dist = d.distance(coord1, coord2).m
    return dist

def output(initial, current, target, waypoint):
    say(waypoint[current])
    initPos = initial
    
    currentPosition = updateGPS() - initPos
    
    asyncio.sleep(2)
    