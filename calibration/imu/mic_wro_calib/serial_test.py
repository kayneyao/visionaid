import serial

ser = serial.Serial('COM6', 115200)

while True:
    line = ser.readline().decode('utf-8').rstrip().split(',')
    print("x: " + line[0] + " y: " + line[1] + " z: " + line[2])