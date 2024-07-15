import time
import sys
sys.path.insert(0, 'aiy-maker-kit/examples')
import detect_objects
import speech
import RPi.GPIO as GPIO


start = time.time()

try:
    while True:
        
        current = time.time()
        
        if ((current-start)%1==0):
            message = detect_objects.detect()
            speech.speak(message)
            
            
            
except KeyboardInterrupt:
    print("Runtime stopped by User")
    

finally:
    GPIO.cleanup()