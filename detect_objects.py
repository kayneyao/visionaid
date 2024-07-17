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


###FUNCTIONS###

#function for predicting traffic light color 
def predict_color(frame):
    
    color_found = 'undefined'
    
    # Color thresholds.

    color_list = [
        ['Red', [0, 120, 70], [10, 255, 255]],
        
        ['Green / Off', [50, 5, 150], [86, 250, 250]],
        ['Red', [170, 120, 70], [180, 255, 255]]
    ]
    
        
    # Change to HSV spectrum.
    
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    
    #find the central region to get rid of the green border
    height, width, _ = hsv_img.shape
    
    center_x, center_y = width//2, height//2
    region_width, region_height = int(width*0.5), int(height*0.5)
    x2, y2 = center_x + region_width//2, center_y + region_height//2
    x1, y1 = center_x - region_width//2, center_y - region_height//2
    
    centralregion = hsv_img[y1:y2, x1:x2]

    # The state of the light is the one with the greatest number of white pixels.
        
    max_count = 0
    
    for color_name, lower_val, upper_val in color_list:
        # Threshold the HSV image - any matching color will show up as white.
        mask = cv2.inRange(centralregion, np.array(lower_val), np.array(upper_val))
        # Count white pixels on mask. Find the color with the most white pixels
        count = np.sum(mask)
        if count > max_count:
            color_found = color_name
            max_count = count
            vision.save_frame("/home/sophie/aiy-maker-kit/data/collect_tl/images/mask.jpg",mask)
            
            
            
    if max_count < 1600:  # Arbitrary threshold to define when it's off (rare cases, mostly at night).
        color_found = "Green / Off"
    
    lightColor = color_found

    if lightColor == 'red':
        class_id = 'Red'

    elif lightColor == 'green':
        class_id = 'Green'
    print(lightColor)

    return lightColor


def collect_colors(frame):
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return hsv_img


def img_data(frame, filename):
      #save tl image to folder
    
    hsv_img = str(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))

    directory = "/home/sophie/aiy-maker-kit/data/collect_tl"
    os.chdir(directory)
    print(os.listdir(directory))
    cv2.putText(frame, hsv_img,
                    (2,10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1,
                    color=(255,0,0), thickness=2)
    cv2.imwrite(filename +'.jpg', frame)
    
    
def write_json(frame):


    hsv_img = str(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
  
        
#     avg_hsv = np.mean(hsv_img, axis=(0,1))
#     
#     
#     h,s,v = int(avg_hsv[0]), int(avg_hsv[1]), int(avg_hsv[2])
#     #create a dictionary
#     hsv_color = {
#       
#         "hue": h,
#         "saturation": s,
#         "value":v,
#         "color":predict_color(frame)
#         }

    hsv_color = {
        "color":hsv_img,
        "predict":predict_color(frame)
        }
   
    #write the file        
    file_path='/home/sophie/aiy-maker-kit/data/collect_tl/json/color_data.json'
    with open(file_path, 'a') as outfile:
        print("writing file to: ",file_path)
        # HERE IS WHERE THE MAGIC HAPPENS
    
        json.dump(hsv_color, outfile, indent=2)
    outfile.close()     
    print("done")
    


###START
###OBJ DETECTION
    
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

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
                    
        
        if 'bicycle' in label:
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


