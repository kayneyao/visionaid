import argparse
import cv2
import glob
import matplotlib
import numpy as np
import os
import torch
import time

from depth_anything_v2.dpt import DepthAnythingV2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Anything V2')
    
    parser.add_argument('--video-path', type=str, default = '0')
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--outdir', type=str, default='./vis_video_depth')
    
    parser.add_argument('--encoder', type=str, default='vits', choices=['vits', 'vitb', 'vitl', 'vitg'])
    
    parser.add_argument('--pred-only', dest='pred_only', action='store_true', help='only display the prediction')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='do not apply colorful palette')
    
    args = parser.parse_args()
    
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }



    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    depth_anything.load_state_dict(torch.load(f'/home/sophie/Depth-Anything/checkpoints/depth_anything_v2_{args.encoder}.pth', map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()
    print(depth_anything)
    
  
    os.makedirs(args.outdir, exist_ok=True)
    
    margin_width = 50
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')
 
        
        
        # creating the videocapture object 
    # and reading from the input file 
    # Change it to 0 if reading from webcam 
    
    cap = cv2.VideoCapture(0) 
    cv2.namedWindow("Frame")
    # used to record the time when we processed last frame 
    prev_frame_time = 0

    # used to record the time at which we processed current frame 
    new_frame_time = 0

    # Reading the video file until finished 
    while(cap.isOpened()): 

        # Capture frame-by-frame 

        ret, frame = cap.read() 
        frame_width, frame_height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
    
        depth = depth_anything.infer_image(frame, args.input_size)
        depthValue = depth[200,500]
        print(depth[20,500])
        print(depth[20,200])
        cv2.imshow('Frame', frame)
        # if video finished or no Video Input 
        if not ret: 
            break

        # Our operations on the frame come here 
        gray = frame 

        # resizing the frame size according to our need 
        gray = cv2.resize(gray, (500, 300)) 

        # font which we will be using to display FPS 
        font = cv2.FONT_HERSHEY_SIMPLEX 
        # time when we finish processing for this frame 
        new_frame_time = time.time() 

        # Calculating the fps 

        # fps will be number of frame processed in given time frame 
        # since their will be most of time error of 0.001 second 
        # we will be subtracting it to get more accurate result 
        fps = 1/(new_frame_time-prev_frame_time) 
        prev_frame_time = new_frame_time 

        # converting the fps into integer 
        fps = int(fps) 

        # converting the fps to string so that we can display it on frame 
        # by using putText function 
        fps = str(fps) 

        # putting the FPS count on the frame 
        cv2.putText(gray, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA) 

        # displaying the frame with fps 
        cv2.imshow('frame', gray) 
        
        # press 'Q' if you want to exit 
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    # When everything done, release the capture 
        cap.release() 
        # Destroy the all windows now 
        cv2.destroyAllWindows() 