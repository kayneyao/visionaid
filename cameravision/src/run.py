import time
#AIY Maker Kit Library
from aiymakerkit import vision
from aiymakerkit import utils
#Local Imports
import sys
sys.path.insert(0, '/home/sophie/visionaid/cameravision/src')
import models

from features import detect_objects
from features import speech
#pip libraries
import RPi.GPIO as GPIO

try:
    start = time.time()
    detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
    lab = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
    while True:
        #to turn camera off, edit get_frames
        
        for frame in vision.get_frames():
            things = detector.get_objects(frame, threshold=0.4)
            vision.draw_objects(frame, things, lab)
            
            current = time.time()
         
            if ((current-start)%3<=0.5) and ((current-start)>3.0):
                print('start')
                message = detect_objects.detect(things, lab, frame)
                print('check')
#                 speech.speak(message)
            
            
            
except KeyboardInterrupt:
    print("Runtime stopped by User")
    

finally:
    GPIO.cleanup()