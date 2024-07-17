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

from aiymakerkit import vision
from aiymakerkit import utils
import models
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import os
import json



###START
###OBJ DETECTION
    
detector = vision.Detector(models.CUSTOM_OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.CUSTOM_OBJECT_DETECTION_MODEL)

for frame in vision.get_frames():
    objects = detector.get_objects(frame, threshold=0.4)
    vision.draw_objects(frame, objects, labels)
    print(labels)
    
    
 #if specific classes are detected, print
    for obj in objects:
        label = labels.get(obj.id)
        xpos = (obj.bbox.xmin + obj.bbox.xmax)/2
        ypos = (obj.bbox.ymin + obj.bbox.ymax)/2
        
        
        #find where the object is
        if 'crosswalk' in label:
            #if person is on the right right
            if (xpos <100):
            
                print("cross at 2 o'clock")
                
            #if person is on the right
            if (xpos<200 and xpos>=100):
            
                print("cross at 1 o'clock")
            #if person is straight
            if (xpos>=200 and xpos <=400):
                    print("Person at 12 o'clock")
                
            #if person is on the left
            if (xpos >400 and xpos <500):
                    print("Person at 11 o'clock")
                    
            #if person is on the left left
            if (xpos>600):
                    print("Person at 10 o'clock")
                    
        
        if 'red' in label:
            #if person is on the right right
            if (xpos <100):
            
                print("Bike at 2 o'clock")
                
            #if person is on the right
            if (xpos<200 and xpos>=100):
            
                print("Bike at 1 o'clock")
            #if person is straight
            if (xpos>=200 and xpos <=400):
                    print("Bike at 12 o'clock")
                
            #if person is on the left
            if (xpos >400 and xpos <500):
                    print("Bike at 11 o'clock")
                    
            #if person is on the left left
            if (xpos>600):
                    print("Bike at 10 o'clock")
                    
                    

            
        if 'traffic light' in label:
            tl = frame[obj.bbox.ymin:obj.bbox.ymax, obj.bbox.xmin:obj.bbox.xmax]
                
            img_data(tl, "trafficlight")
            write_json(tl)
            
            prediction = predict_color(tl)
            print(collect_colors(tl))
            #if traffic light is green
            if prediction == 'Green / Off':
                color = (0, 255, 0)
                print("Traffic light is green")
            elif prediction == 'Red':
                color = (0, 0, 255)
                print("Traffic light is red")
         
            
            #if TL is on the right right
            if (xpos <100):
            
                print("Traffic light at 2 o'clock")
                
            #if tl is on the right
            elif (xpos<200 and xpos>=100):
            
                print("Traffic light at 1 o'clock")
            #if tl is straight
            elif (xpos>=200 and xpos <=400):
                    print("Traffic light at 12 o'clock")
                
            #if tl is on the left
            elif (xpos >400 and xpos <500):
                    print("Traffic light at 11 o'clock")
                    
            #if tl is on the left left
            elif (xpos>600):
                    print("Traffic light at 10 o'clock")



