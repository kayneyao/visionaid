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
sys.path.insert(0, 'aiy-maker-kit/examples')
from aiymakerkit import vision
from aiymakerkit import utils
import ultrasonic
import trafficlights
from num2words import num2words
import speech
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

def detect():
    
    message = ['']
    
    detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
    labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
    found = False
    for frame in vision.get_frames():
        objects = detector.get_objects(frame, threshold=0.4)
        vision.draw_objects(frame, objects, labels)
        
        ##ULTRASONIC SENSOR MAIN        
        
        
     #if specific classes are detected, print
        for obj in objects:
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
                    
                elif prediction == 'Red':
                    color = (0, 0, 255)
                    message.append("Traffic light is red")
                    
             
                
#                 #if TL is on the right right
#                 if (xpos <100):
#                 
#                     print("Traffic light at 2 o'clock")
#                     
#                 #if tl is on the right
#                 elif (xpos<200 and xpos>=100):
#                 
#                     print("Traffic light at 1 o'clock")
#                 #if tl is straight
#                 elif (xpos>=200 and xpos <=400):
#                         print("Traffic light at 12 o'clock")
#                     
#                 #if tl is on the left
#                 elif (xpos >400 and xpos <500):
#                         print("Traffic light at 11 o'clock")
#                         
#                 #if tl is on the left left
#                 elif (xpos>600):
#                         print("Traffic light at 10 o'clock")
#                         
        
            if 'person' in label:
                found = True
                
                if (xpos <100):
                #if person is on the right
                    distance = ultrasonic.Rdistance()
                    distance = num2words(distance)
               
                    message.append('Person')
                    message.append(distance)
                    message.append("centimeters away, at 2 oclock")
               
                    break
           
                   
                #if person is on the right
                elif (xpos<200 and xpos>=100):

                    distance = ultrasonic.Rdistance()
                    distance = num2words(distance)
               
                    message.append('Person')
                    message.append(distance)
                    message.append("centimeters away, at 1 oclock")
                    break
            
                    
                #if person is straight
                elif (xpos>=200 and xpos <=400):
                    message.append("Person straight infront")
                    break
          
                #if person is on the left
                elif (xpos >400 and xpos <500):
                    distance = ultrasonic.Ldistance()
                    distance = num2words(distance)
                    message.appned('Person')
                    message.append(distance)
                    message.append("centimeters away, at 11 oclock")
                    break
            
                    
                #if person is on the left left
                elif (xpos>600):
                    distance = ultrasonic.Ldistance()
                    distance = num2words(distance)
                    message.append('Person')
                    message.append(distance)
                    message.append("centimeters away, at 10 oclock")    
                    break
                    
                
                
                
            if 'bicycle' in label:
                
                found = True
                if (xpos <100):
                #if B is on the right
                    distance = ultrasonic.Rdistance()
                    distance = num2words(distance)
                    message += ('Bike', distance, "centimeters away, at 2 oclock")
                    break
           
                   
                #if B is on the right
                elif (xpos<200 and xpos>=100):

                    distance = ultrasonic.Rdistance()
                    distance = num2words(distance)
                    message += ('Bike', distance, "centimeters away, at 1 oclock")
                    break
            
                    
                #if B is straight
                elif (xpos>=200 and xpos <=400):
                    message += "Bike straight infront"
                    break
          
                #if B is on the left
                elif (xpos >400 and xpos <500):
                    distance = ultrasonic.Ldistance()
                    distance = num2words(distance)
                    message += 'Bike', distance, "centimeters away, at 11 oclock"
                    break
            
                    
                #if B is on the left left
                elif (xpos>600):
                    distance = ultrasonic.Ldistance()
                    distance = num2words(distance)
                    message += 'Bike', distance, "centimeters away, at 10 oclock"
                    break
            
            if (found==True):
                print(found)
                break
            
            else:
                print("Nothing found")
        if (found==True):
            break
    
    return message  
        
        

