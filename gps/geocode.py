from geopy.geocoders import Nominatim

agent = Nominatim(user_agent="Geopy Library")

address = input()

location = agent.geocode(address)

print([location.latitude, location.longitude])