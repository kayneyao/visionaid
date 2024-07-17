#traffic light 
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
    x1, y1 = center_x - region_width//2, center_y -region_height//2
    x2, y2 = center_x + region_width//2, center_y +region_height//2
    
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
    