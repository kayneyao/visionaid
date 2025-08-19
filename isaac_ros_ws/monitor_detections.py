#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import time
from datetime import datetime

class DetectionMonitor(Node):
    def __init__(self):
        super().__init__('detection_monitor')
        
        # 11-class mapping
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Safety priorities for display
        self.safety_priorities = {
            'vehicles': [1, 2, 5, 9],           # bus, car, motorcycle, truck
            'traffic_lights': [4, 7, 10],      # greenlight, redlight, yellowlight
            'pedestrian_context': [0, 6],      # bicycle, pedestrian
            'infrastructure': [3, 8]           # crosswalk, sidewalk
        }
        
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detection_callback,
            10)
        
        print("Detection Monitor Started!")
        print("Monitoring /camera/detections for all 11 classes...")
        print("Will show detections with timestamps and confidence")
        print("Press Ctrl+C to stop\n")
        
    def get_priority_group(self, class_id):
        """Get the priority group for a class"""
        if class_id in self.safety_priorities['vehicles']:
            return "PRIORITY 1 (VEHICLES)"
        elif class_id in self.safety_priorities['traffic_lights']:
            return "PRIORITY 2 (TRAFFIC LIGHTS)"
        elif class_id in self.safety_priorities['pedestrian_context']:
            return "CONTEXT (PEDESTRIANS)"
        elif class_id in self.safety_priorities['infrastructure']:
            return "CONTEXT (INFRASTRUCTURE)"
        else:
            return "UNKNOWN"
    
    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[{timestamp}] DETECTIONS FOUND: {len(msg.detections)} object(s)")
            print("=" * 60)
            
            for i, detection in enumerate(msg.detections):
                if detection.results:
                    class_id = int(detection.results[0].hypothesis.class_id)
                    confidence = detection.results[0].hypothesis.score
                    class_name = self.class_names.get(class_id, f'unknown_{class_id}')
                    priority = self.get_priority_group(class_id)
                    
                    # Get bounding box info
                    bbox = detection.bbox
                    center_x = bbox.center.position.x
                    center_y = bbox.center.position.y
                    width = bbox.size_x
                    height = bbox.size_y
                    
                    print(f"  Detection #{i+1}:")
                    print(f"    Class: {class_name} (ID: {class_id})")
                    print(f"    Confidence: {confidence:.1%}")
                    print(f"    Priority: {priority}")
                    print(f"    BBox: center=({center_x:.0f},{center_y:.0f}), size=({width:.0f}x{height:.0f})")
                    print()
            
            print("=" * 60)
            
            # Add a small delay to make it more readable
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    
    monitor = DetectionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nDetection monitoring stopped.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 