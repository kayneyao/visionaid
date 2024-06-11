#Library Imports
import routingpy as rp
import json

#Instantiate Variables
coords = [[8.512516, 47.380742], [8.557835, 47.359467]]
key_ors = "5b3ce3597851110001cf62483c18fce726724c5a8705062fdb07ad7e"

#Instantiate OpenRouteService API Client
api = rp.ORS(api_key=key_ors)

#Request directions from API
route = api.directions(
    locations=coords,
    profile='foot-walking',
    format='json', 
    units='m',
    language='en',
    instructions=True,
    instructions_format="text")

#Write the raw output of the API's Response to a JSON file
with open('./gps/data.json', 'w') as f:
    json.dump(route.raw, f)

stored_route = json.load(open('./gps/data.json'))
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
    