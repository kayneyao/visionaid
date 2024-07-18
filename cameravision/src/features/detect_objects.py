# Copyright 2021 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Performs continuous object detection with the camera.

Simply run the script and it will draw boxes around detected objects along
with the predicted labels:

    python3 detect_objects.py

For more instructions, see g.co/aiy/maker
"""
import sys
sys.path.insert(0, '/home/sophie/visionaid/cameravision/src')
from aiymakerkit import vision
from aiymakerkit import utils
from features import ultrasonic
from features import trafficlights
from num2words import num2words
import models
from picamera.array import PiRGBArray
from picamera import PiCamera


#traffic light 
import cv2
import numpy as np
import os
import json


#ultrasonic sensor
import RPi.GPIO as GPIO
import time

##SETUP


    
###START

        
            
        ###OBJ DETECTION

def detect(objects, labels, frame):
    
    
    
#     detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
#     labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
    found = False
    message = ['']
#     for frame in vision.get_frames():
#         objects = detector.get_objects(frame, threshold=0.4)
#         vision.draw_objects(frame, objects, labels)
        
        ##ULTRASONIC SENSOR MAIN        
        
        
     #if specific classes are detected, print
    for obj in objects:
        message = ['']
        label = labels.get(obj.id)
        xpos = (obj.bbox.xmin + obj.bbox.xmax)/2
        ypos = (obj.bbox.ymin + obj.bbox.ymax)/2

        if 'traffic light' in label:
    
            found = True
            tl = frame[obj.bbox.ymin:obj.bbox.ymax, obj.bbox.xmin:obj.bbox.xmax]
                
#                   img_data(tl, "trafficlight")
#                   write_json(tl)
            
            prediction = trafficlights.predict_color(tl)
            print(trafficlights.collect_colors(tl))
            #if traffic light is green
            if prediction == 'Green / Off':
                color = (0, 255, 0)
                message.append("Traffic light is green")
                break
   
            elif prediction == 'Red':
                color = (0, 0, 255)
                message.append("Traffic light is red") 
                break
                
     
     
        if ('person' in label) or ('bicycle' in label):
            found = True
            
            if (xpos <100):
            #if person is on the right
#                 distance = round(ultrasonic.Rdistance() / 100, 2)
                direction = '2 oclock'
               
            #if person is on the right
            elif (xpos<200 and xpos>=100):
#                 distance = round(ultrasonic.Rdistance() / 100, 2)
                direction = '1 oclock'
      
            #if person is on the left
            elif (xpos >400 and xpos <500):
#                 distance = round(ultrasonic.Ldistance() / 100, 2)
                direction = '11 oclock'
                
            #if person is on the left left
            elif (xpos>600):
#                 distance = round(ultrasonic.Ldistance() / 100, 2)
                direction = '10 oclock'
            
            if (xpos<=200 or xpos >=400):
                print(direction)
#                 distance = num2words(distance)
                message.append(label)
                message.append(distance)
                message.append("meters away, at " + direction)
                break
            else:
                message.append(label)
                message.append("straight infront")
                break
        
            
    if (found==True):
        print(found)

    else:
        print('Nothing Found')
 
    return message
        
        

