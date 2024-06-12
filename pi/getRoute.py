import routingpy as rp
from geopy.geocoders import Nominatim
import json
import pynmea2
import serial

key_ors = '5b3ce3597851110001cf62483c18fce726724c5a8705062fdb07ad7e'
api = rp.ORS(api_key=key_ors)

agent = Nominatim(user_agent='Geopy Library')

#ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5.0)

# def updateCoord():
# 	dataout = pynmea2.NMEAStreamReader()
# 	gps = [0,0]
# 	while 1:
import asyncio
# 		newdata = ser.readline()
# 		n_data = newdata.decode('latin-1')
# 		if n_data[:6] == '$GNRMC':
# 			newmsg = pynmea2.parse(n_data)
# 			lat = newmsg.latitude
# 			lng = newmsg.longitude
# 			gps = [lng,lat]
# 			print(gps)
# 			return gps

def getDestination():
    address = input()
    location = agent.geocode(address)

    print(location)

    while location == None:
        print("Please try again with a different input")
        address = input()
        location = agent.geocode(address)
    
    destination = [location.longitude, location.latitude]

    print(destination)

    return destination
    

def getDirections():
    coords = [getDestination(), getDestination()]

    route = api.directions(
        locations=coords,
        profile='foot-walking',
        format='json', 
        units='m',
        language='en',
        instructions=True,
        instructions_format='text')

    with open('./routes/directions.json', 'w') as f:
        json.dump(route.raw, f)


getDirections()
