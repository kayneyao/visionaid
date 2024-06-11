import routingpy as rp
from geopy.geocoders import Nominatim
import json
import pynmea2
import serial

key_ors = '5b3ce3597851110001cf62483c18fce726724c5a8705062fdb07ad7e'
api = rp.ORS(api_key=key_ors)

agent = Nominatim(user_agent='Geopy Library')

# ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5.0)

def updateCoord():
	dataout = pynmea2.NMEAStreamReader()
	gps = [0,0]
	while 1:
		newdata = ser.readline()
		n_data = newdata.decode('latin-1')
		if n_data[:6] == '$GNRMC':
			newmsg = pynmea2.parse(n_data)
			lat = newmsg.latitude
			lng = newmsg.longitude
			gps = [lat,lng]
			print(gps)
			return gps

def getDestination():
    address = input()
    location = agent.geocode(address)

    while type(location) == None:
        print("Please try again with a different input")
        address = input()
        location = agent.geocode(address)
    
    destination = [location.latitude, location.longitude]

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

    with open('directions.json', 'w') as f:
        json.dump(route.raw, f)

    stored_route = json.load(open('directions.json'))
    instruction = stored_route.get('routes')[0].get('segments')[0].get('steps')

    for parse in instruction:
        output = str(parse.get('instruction'))
        if str(parse.get('instruction'))[:6] == 'Arrive':
            print(output)
        elif str(parse.get('instruction'))[:4] != 'Turn':
            output = output + ' for ' + str(parse.get('distance')) + ' meters'
            print(output)
        else:
            print(output)
            print('And head straight for ' + str(parse.get('distance')) + ' meters')


getDirections()
