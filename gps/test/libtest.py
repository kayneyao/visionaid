# This snippet has been automatically generated and should be regarded as a
# code template only.
# It will require modifications to work:
# - It may require correct/in-range values for request initialization.
# - It may require specifying regional endpoints when creating the service
#   client as shown in:
#   https://googleapis.dev/python/google-api-core/latest/client_options.html

import asyncio
from google.maps import routing_v2
from google.type import latlng_pb2
from typing import Sequence, Tuple


lngI = 51
latI = 52
lngF = -100
latF = 45

apikey = 'AIzaSy1Cg1V_kKVNuqXG5XLzrnPtg6LIBZOXcP9U'

def sample_compute_routes():
    # Create a client
    client = routing_v2.RoutesClient()
    fieldMask = {('X-Goog-FieldMask: routes.distanceMeters', 'X-Goog-API-Key: ' + apikey),}

    # Initialize request argument(s)
    request = routing_v2.ComputeRoutesRequest(
        origin=routing_v2.types.Waypoint(
            location=routing_v2.types.Location(
                lat_lng=latlng_pb2.LatLng(latitude=latI, longitude=lngI)
            )
        ),
        destination=routing_v2.types.Waypoint(
            location=routing_v2.types.Location(
                lat_lng=latlng_pb2.LatLng(latitude=latF, longitude=lngF)
            )
        )
    )

    # Make the request
    response = client.compute_routes(
        request=request, 
        metadata=fieldMask
        )

    # Handle the response
    print(response)

sample_compute_routes()