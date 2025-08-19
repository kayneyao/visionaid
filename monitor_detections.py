#!/usr/bin/env python3
"""
Detection Monitoring Script for YOLOv8 Traffic Safety System
Monitors detection topics and provides real-time statistics
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

class DetectionMonitor(Node):
    def __init__(self, show_viz=False, stats_only=False):
        super().__init__('detection_monitor')
        
        self.show_viz = show_viz
        self.stats_only = stats_only
        self.bridge = CvBridge()
        
        # 11-Class mapping for Taiwan traffic safety system
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Class-specific confidence thresholds (matching YOLOv8 node)
        self.class_confidence_thresholds = {
            0: 0.5,   # bicycle
            1: 0.6,   # bus
            2: 0.5,   # car
            3: 0.5,   # crosswalk
            4: 0.6,   # greenlight
            5: 0.5,   # motorcycle
            6: 0.5,   # pedestrian
            7: 0.6,   # redlight
            8: 0.5,   # sidewalk
            9: 0.6,   # truck
            10: 0.6   # yellowlight
        }
        
        # Statistics tracking
        self.detection_stats = defaultdict(int)
        self.latest_confidences = {}  # Track latest confidence for each class
        self.fps_stats = deque(maxlen=30)  # Last 30 frames
        self.last_time = time.time()
        self.frame_count = 0
        self.reset_interval = 30  # Reset stats every 30 frames
        
        # Track filtered detections
        self.filtered_detection_count = 0
        self.last_published_count = 0
        self.filtered_classes = defaultdict(int)  # Track which classes are being filtered
        self.raw_detection_stats = defaultdict(int)  # Track raw detections from visualization
        self.last_filtering_log_time = 0  # Track when we last logged filtering info
        self.filtering_log_interval = 5.0  # Only log filtering every 5 seconds
        self.last_detection_time = time.time()  # Track when we last saw any detection
        self.no_detection_threshold = 10.0  # Show "no detections" after 10 seconds
        
        # Subscribe to detection topics
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detection_callback,
            10
        )
        
        # Subscribe to raw detections (before filtering) - we'll need to create this topic
        self.raw_detection_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections/raw',  # This topic needs to be created in YOLOv8 node
            self.raw_detection_callback,
            10
        )
        
        # Always subscribe to visualization to monitor filtered detections
        self.viz_sub = self.create_subscription(
            Image,
            '/camera/detections/visualization',
            self.viz_callback,
            10
        )
        
        if self.show_viz:
            cv2.namedWindow('YOLOv8 Detections', cv2.WINDOW_NORMAL)
        
        self.get_logger().info('Detection Monitor Started (with Temporal Filter)')
        self.get_logger().info('Available monitoring modes:')
        self.get_logger().info('   - Detection messages and statistics')
        self.get_logger().info('   - Real-time FPS tracking')
        self.get_logger().info('   - Class-wise detection counts')
        self.get_logger().info('   - Confidence threshold status')
        self.get_logger().info('   - Temporal filter information')
        if self.show_viz:
            self.get_logger().info('   - Visual detection display')
    
    def detection_callback(self, msg):
        """Process detection messages and update statistics"""
        current_time = time.time()
        self.frame_count += 1
        
        # Calculate FPS
        if self.last_time > 0:
            fps = 1.0 / (current_time - self.last_time)
            self.fps_stats.append(fps)
        
        self.last_time = current_time
        
        # Track published detections
        self.last_published_count = len(msg.detections)
        
        # Reset statistics periodically to show current detections
        if self.frame_count % self.reset_interval == 0:
            self.detection_stats.clear()
            self.latest_confidences.clear()
            self.filtered_classes.clear()
            self.raw_detection_stats.clear()
        
        # Process detections
        if msg.detections:
            self.last_detection_time = current_time  # Update detection time
        
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Update statistics
                self.detection_stats[class_id] += 1
                # Store latest confidence for this class
                self.latest_confidences[class_id] = confidence
                
                if not self.stats_only:
                    class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                    threshold = self.class_confidence_thresholds.get(int(class_id), 0.5)
                    threshold_status = "PASS" if confidence >= threshold else "FAIL"
                    self.get_logger().info(f'Detection: {class_name} (Class {class_id}), Confidence: {confidence:.3f} (Threshold: {threshold:.2f} - {threshold_status})')
                
                # Special logging for traffic lights
                if int(class_id) in [4, 7, 10]:  # greenlight, redlight, yellowlight
                    self.get_logger().info(f'Traffic light published: {class_name} (Class {class_id}) - Confidence: {confidence:.3f}')
        
        # Display statistics
        self.display_stats()
    
    def raw_detection_callback(self, msg):
        """Process raw detection messages (before filtering) to track what's being filtered out"""
        # Track raw detections by class
        raw_detections_this_frame = defaultdict(int)
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                
                # Store raw detection info
                self.raw_detection_stats[class_name] += 1
                self.raw_detection_stats[f'{class_name}_confidence'] = confidence
                raw_detections_this_frame[class_name] += 1
        
        # Compare with published detections to identify filtered classes
        published_classes = set()
        for class_id, count in self.detection_stats.items():
            if count > 0:
                class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                published_classes.add(class_name)
        
        # Find filtered classes (in raw but not in published)
        current_time = time.time()
        filtered_this_frame = []
        
        for class_name, count in raw_detections_this_frame.items():
            if class_name not in published_classes:
                self.filtered_classes[class_name] += count
                confidence = self.raw_detection_stats.get(f'{class_name}_confidence', 0)
                filtered_this_frame.append((class_name, confidence))
        
        # Only log filtering info periodically to reduce spam
        if filtered_this_frame and (current_time - self.last_filtering_log_time) > self.filtering_log_interval:
            if not self.stats_only:
                filtered_details = ', '.join([f"{name} (conf: {conf:.3f})" for name, conf in filtered_this_frame])
                self.get_logger().info(f'Filtered Classes: {filtered_details}')
            self.last_filtering_log_time = current_time
    
    def viz_callback(self, msg):
        """Display visualization image and track filtered detections"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Count colored bounding boxes in the image to estimate raw detections
            # Look for colored rectangles (detection boxes)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Count potential detection boxes (rectangles with reasonable size)
            potential_detections = 0
            filtered_by_class = defaultdict(int)
            
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if 20 < w < 400 and 20 < h < 300:  # Reasonable detection box size
                    potential_detections += 1
                    
                    # Try to extract class name from text label above the bounding box
                    # Look for text in the area above the bounding box
                    text_roi_y1 = max(0, y - 30)
                    text_roi_y2 = max(0, y - 5)
                    text_roi_x1 = max(0, x - 10)
                    text_roi_x2 = min(cv_image.shape[1], x + w + 10)
                    
                    if text_roi_y1 < text_roi_y2 and text_roi_x1 < text_roi_x2:
                        text_roi = cv_image[text_roi_y1:text_roi_y2, text_roi_x1:text_roi_x2]
                        
                        # Simple text detection - look for areas with high contrast
                        gray_text = cv2.cvtColor(text_roi, cv2.COLOR_BGR2GRAY)
                        _, binary = cv2.threshold(gray_text, 127, 255, cv2.THRESH_BINARY)
                        
                        # If we find text-like patterns, try to identify the class
                        # Simplified approach; OCR would be used in production
                        # For now, use color-based classification as fallback
                        if y > 20:  # Make sure we're not sampling outside image
                            sample_y = max(0, y - 15)
                            sample_x = max(0, x)
                            if sample_y < cv_image.shape[0] and sample_x < cv_image.shape[1]:
                                color = cv_image[sample_y, sample_x]
                                # Basic color classification (BGR format)
                                if color[2] > 200 and color[1] < 100 and color[0] < 100:  # Red dominant
                                    filtered_by_class['red_objects'] += 1
                                elif color[1] > 200 and color[2] < 100 and color[0] < 100:  # Green dominant
                                    filtered_by_class['green_objects'] += 1
                                elif color[1] > 200 and color[2] > 200 and color[0] < 100:  # Yellow dominant
                                    filtered_by_class['yellow_objects'] += 1
                                elif color[0] > 200 and color[1] < 100 and color[2] < 100:  # Blue dominant
                                    filtered_by_class['blue_objects'] += 1
                                else:
                                    filtered_by_class['other_objects'] += 1
            
            # Track raw detections (all boxes visible in visualization)
            self.raw_detection_stats['total_boxes'] += potential_detections
            
            # If we see potential detections but no published detections, they're filtered
            if potential_detections > 0 and self.last_published_count == 0:
                self.filtered_detection_count += 1
                self.filtered_classes['total_filtered'] += potential_detections
                
                # Track filtered objects by color/class
                for class_type, count in filtered_by_class.items():
                    self.filtered_classes[class_type] += count
            
            if self.show_viz:
                cv2.imshow('YOLOv8 Detections', cv_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing visualization: {e}')
    
    def display_stats(self):
        """Display current statistics with temporal filter information"""
        if not self.detection_stats:
            return
        
        # Calculate average FPS
        avg_fps = np.mean(self.fps_stats) if self.fps_stats else 0
        
        print("\n" + "="*60)
        print("YOLOv8 DETECTION STATISTICS (with Temporal Filter)")
        print("="*60)
        print(f"Average FPS: {avg_fps:.1f}")
        print("Temporal Filter: 2 consecutive frames required")
        print("Confidence Thresholds: Lowered for temporal filtering")
        print("\nCurrently Detected Classes (Temporally Consistent):")
        
        # Sort by detection count
        sorted_stats = sorted(self.detection_stats.items(), key=lambda x: x[1], reverse=True)
        
        for class_id, count in sorted_stats:
            class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
            # Get the latest confidence for this class
            latest_confidence = self.get_latest_confidence(class_id)
            confidence_str = ""
            if latest_confidence:
                threshold = self.class_confidence_thresholds.get(int(class_id), 0.5)
                threshold_status = "PASS" if latest_confidence >= threshold else "FAIL"
                confidence_str = f" (conf: {latest_confidence:.3f}, thresh: {threshold:.2f} - {threshold_status})"
            print(f"   {class_name} (Class {class_id}): {count} detections{confidence_str}")
        
        print(f"\nFiltered Detections: {self.filtered_detection_count} frames with filtered detections")
        print(f"Published Detections: {self.last_published_count} (temporally consistent)")
        
        # Show filtering statistics with actual class names
        if self.filtered_classes:
            print(f"Total Filtered Objects: {self.filtered_classes.get('total_filtered', 0)}")
            # Show class-specific filtering (excluding color-based classifications)
            filtered_details = []
            for class_name, count in self.filtered_classes.items():
                if (class_name != 'total_filtered' and 
                    class_name not in ['red_objects', 'green_objects', 'yellow_objects', 'blue_objects', 'other_objects'] and 
                    count > 0):
                    confidence = self.raw_detection_stats.get(f'{class_name}_confidence', 0)
                    filtered_details.append(f"{class_name}: {count} (conf: {confidence:.3f})")
            if filtered_details:
                print(f"Filtered Classes: {', '.join(filtered_details)}")
            else:
                print("Filtered Classes: None (all detections passed filters)")
        if self.raw_detection_stats:
            print(f"Total Raw Detections: {self.raw_detection_stats.get('total_boxes', 0)}")
        
        print("\nFiltering System:")
        print("   • Confidence Threshold: First filter (lowered to 0.5-0.6)")
        print("   • Temporal Consistency: Second filter (2 consecutive frames)")
        print("   • Only temporally consistent detections are published")
        
        # Traffic light specific statistics
        traffic_light_stats = {}
        for class_id, count in self.detection_stats.items():
            if int(class_id) in [4, 7, 10]:  # greenlight, redlight, yellowlight
                class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                confidence = self.get_latest_confidence(class_id)
                traffic_light_stats[class_name] = (count, confidence)
        
        if traffic_light_stats:
            print("\nTRAFFIC LIGHT DETECTIONS:")
            for class_name, (count, confidence) in traffic_light_stats.items():
                print(f"   {class_name}: {count} detections (latest conf: {confidence:.3f})")
        # Show "no detections" message only after a longer period
        current_time = time.time()
        if not traffic_light_stats and (current_time - self.last_detection_time) > self.no_detection_threshold:
            print("\nTRAFFIC LIGHT DETECTIONS: No traffic lights detected for 10+ seconds")
        
        print("="*60)
        print("Press Ctrl+C to stop monitoring")
        print("="*60)
    
    def get_latest_confidence(self, class_id):
        """Get the latest confidence score for a given class"""
        return self.latest_confidences.get(class_id, None)

def main():
    parser = argparse.ArgumentParser(description='Monitor YOLOv8 detections')
    parser.add_argument('--viz', action='store_true', help='Show visualization window')
    parser.add_argument('--stats-only', action='store_true', help='Show only statistics, no individual detections')
    args = parser.parse_args()
    
    rclpy.init()
    
    monitor = DetectionMonitor(show_viz=args.viz, stats_only=args.stats_only)
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    finally:
        if args.viz:
            cv2.destroyAllWindows()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 