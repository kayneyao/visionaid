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
from pycoral.adapters.detect import BBox
import models
import cv2
import torch

from depth_anything_v2.dpt import DepthAnythingV2

detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

for frame in vision.get_frames():
    objects = detector.get_objects(frame, threshold=0.4)
    vision.draw_objects(frame, objects, labels)
    
    
    #if specific classes are detected, print
    for obj in objects:
        label = labels.get(obj.id)
        xpos = (obj.bbox.xmin + obj.bbox.xmax)/2
        ypos = (obj.bbox.ymin + obj.bbox.ymax)/2
        print(label)
        
        #find where the object is
        if 'person' in label:
            #if person is on the right right
            if (xpos <100):
            
                print("Person at 2 o'clock")
                
            #if person is on the right
            if (xpos<200 and xpos>=100):
            
                print("Person at 1 o'clock")
            #if person is straight
            if (xpos>=200 and xpos <=400):
                    print("Person at 12 o'clock")
                
            #if person is on the left
            if (xpos >400 and xpos <500):
                    print("Person at 11 o'clock")
                    
            #if person is on the left left
            if (xpos>600):
                    print("Person at 10 o'clock")
                    
        
        if 'car' in label:
            #if person is on the right right
            if (xpos <100):
            
                print("Car at 2 o'clock")
                
            #if person is on the right
            if (xpos<200 and xpos>=100):
            
                print("Car at 1 o'clock")
            #if person is straight
            if (xpos>=200 and xpos <=400):
                    print("Person at 12 o'clock")
                
            #if person is on the left
            if (xpos >400 and xpos <500):
                    print("Car at 11 o'clock")
                    
            #if person is on the left left
            if (xpos>600):
                    print("Car at 10 o'clock")
