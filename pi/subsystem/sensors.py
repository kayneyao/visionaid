import serial
import pynmea2



class sensors(object):
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5.0)
        self.system = sensors()
    
    def updateCoord():
        coord = [0,0]
        while 1:
            newdata = self.ser.readline()
            n_data = newdata.decode('latin-1')
            if n_data[:6] == '$GNGGA':
                newmsg = pynmea2.parse(n_data)
                lat = newmsg.latitude
                lng = newmsg.longitude
                coord = [lng,lat]
                # print(gps)
                return coord
    def getSystem():
        return self.system