#!/usr/bin/env python3
"""
Detection Monitoring Script with Image Saving - YOLOv8 Traffic Safety System
Saves detection images to files instead of displaying them in a window
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import defaultdict, deque
import argparse
import os
from datetime import datetime

class DetectionMonitorSave(Node):
    def __init__(self, save_images=False, stats_only=False, save_dir="detection_images"):
        super().__init__('detection_monitor_save')
        
        self.save_images = save_images
        self.stats_only = stats_only
        self.save_dir = save_dir
        self.bridge = CvBridge()
        
        # 11-Class mapping for Taiwan traffic safety system
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Statistics tracking
        self.detection_stats = defaultdict(int)
        self.fps_stats = deque(maxlen=30)  # Last 30 frames
        self.last_time = time.time()
        self.frame_count = 0
        
        # Create save directory if needed
        if self.save_images:
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f'Saving detection images to: {os.path.abspath(self.save_dir)}')
        
        # Subscribe to detection topics
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detection_callback,
            10
        )
        
        if self.save_images:
            self.viz_sub = self.create_subscription(
                Image,
                '/camera/detections/visualization',
                self.viz_callback,
                10
            )
        
        self.get_logger().info('Detection Monitor with Image Saving Started')
        self.get_logger().info('Available monitoring modes:')
        self.get_logger().info('   - Detection messages and statistics')
        self.get_logger().info('   - Real-time FPS tracking')
        self.get_logger().info('   - Class-wise detection counts')
        if self.save_images:
            self.get_logger().info('   - Saving detection images to files')
    
    def detection_callback(self, msg):
        """Process detection messages and update statistics"""
        current_time = time.time()
        self.frame_count += 1
        
        # Calculate FPS
        if self.last_time > 0:
            fps = 1.0 / (current_time - self.last_time)
            self.fps_stats.append(fps)
        
        self.last_time = current_time
        
        # Process detections
        detections_found = False
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Update statistics
                self.detection_stats[class_id] += 1
                detections_found = True
                
                if not self.stats_only:
                    class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                    self.get_logger().info(f'Detection: {class_name} (Class {class_id}), Confidence: {confidence:.3f}')
        
        # Display statistics every 10 frames or when detections are found
        if self.frame_count % 10 == 0 or detections_found:
            self.display_stats()
    
    def viz_callback(self, msg):
        """Save visualization image to file"""
        if self.save_images:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Create filename with timestamp and detection info
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                
                # Get current detection stats for filename
                if self.detection_stats:
                    class_info = []
                    for class_id, count in self.detection_stats.items():
                        class_name = self.class_names.get(int(class_id), f'Unknown{class_id}')
                        class_info.append(f"{class_name}{count}")
                    class_str = "_".join(class_info[-3:])  # Last 3 classes
                else:
                    class_str = "no_detections"
                
                filename = f"detection_{timestamp}_{class_str}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                
                # Save image
                cv2.imwrite(filepath, cv_image)
                
                # Log save info
                self.get_logger().info(f'Saved detection image: {filename}')
                
            except Exception as e:
                self.get_logger().error(f'Error saving visualization: {e}')
    
    def display_stats(self):
        """Display current statistics"""
        if not self.detection_stats:
            return
        
        # Calculate average FPS
        avg_fps = np.mean(self.fps_stats) if self.fps_stats else 0
        
        print("\n" + "="*60)
        print("YOLOv8 DETECTION STATISTICS")
        print("="*60)
        print(f"Average FPS: {avg_fps:.1f}")
        print(f"Total Detections: {sum(self.detection_stats.values())}")
        print(f"Frames Processed: {self.frame_count}")
        if self.save_images:
            print(f"Images Saved: {len(os.listdir(self.save_dir)) if os.path.exists(self.save_dir) else 0}")
        print("\nClass-wise Detections:")
        
        # Sort by detection count
        sorted_stats = sorted(self.detection_stats.items(), key=lambda x: x[1], reverse=True)
        
        for class_id, count in sorted_stats:
            class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
            print(f"   {class_name} (Class {class_id}): {count} detections")
        
        print("="*60)
        print("DETECTION SUMMARY:")
        print(f"   Total Classes Detected: {len(self.detection_stats)}")
        print(f"   Classes Found: {', '.join([self.class_names.get(int(cid), f'Unknown({cid})') for cid in self.detection_stats.keys()])}")
        if self.save_images:
            print(f"   Images Directory: {os.path.abspath(self.save_dir)}")
        print("="*60)
        print("Press Ctrl+C to stop monitoring")
        print("="*60)

def main():
    parser = argparse.ArgumentParser(description='Monitor YOLOv8 detections with image saving')
    parser.add_argument('--save-images', action='store_true', help='Save detection images to files')
    parser.add_argument('--stats-only', action='store_true', help='Show only statistics, no individual detections')
    parser.add_argument('--save-dir', default='detection_images', help='Directory to save detection images')
    args = parser.parse_args()
    
    rclpy.init()
    
    monitor = DetectionMonitorSave(
        save_images=args.save_images, 
        stats_only=args.stats_only,
        save_dir=args.save_dir
    )
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
        if args.save_images:
            print(f"Detection images saved in: {os.path.abspath(args.save_dir)}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 