#!/usr/bin/python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
#     GPIO.setwarnings(False)

PIN_TRIGGER1 = 23
PIN_ECHO1 = 24

PIN_TRIGGER2 = 6
PIN_ECHO2 = 5

def Ldistance():


    GPIO.setup(PIN_TRIGGER1, GPIO.OUT)
    GPIO.setup(PIN_ECHO1, GPIO.IN)

    GPIO.output(PIN_TRIGGER1, GPIO.LOW)

#     print ("Waiting for sensor to settle")

    time.sleep(0.1)

#     print ("Calculating distance")
# 
    GPIO.output(PIN_TRIGGER1, GPIO.HIGH)

    time.sleep(0.0001)

    GPIO.output(PIN_TRIGGER1, GPIO.LOW)

    
    while GPIO.input(PIN_ECHO1)==0:
        pulse_start_time = time.time()
        
        
    while GPIO.input(PIN_ECHO1)==1:
        pulse_end_time = time.time()
            

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    return distance

  
def Rdistance():
    
    GPIO.setup(PIN_TRIGGER2, GPIO.OUT)
    GPIO.setup(PIN_ECHO2, GPIO.IN)

    GPIO.output(PIN_TRIGGER2, GPIO.LOW)

#     print ("Waiting for sensor to settle")

    time.sleep(0.1)

#     print ("Calculating distance")
# 
    GPIO.output(PIN_TRIGGER2, GPIO.HIGH)

    time.sleep(0.0001)

    GPIO.output(PIN_TRIGGER2, GPIO.LOW)

    
    while GPIO.input(PIN_ECHO2)==0:
        pulse_start_time = time.time()
        
        
    while GPIO.input(PIN_ECHO2)==1:
        pulse_end_time = time.time()
            

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    return distance

# 
# Ldistance()
# Rdistance()
