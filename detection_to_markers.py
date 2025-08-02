#!/usr/bin/env python3
"""
Detection to Markers Converter for RViz Visualization
Converts Detection2DArray messages to MarkerArray for RViz display
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np

class DetectionToMarkers(Node):
    def __init__(self):
        super().__init__('detection_to_markers')
        
        # 11-Class mapping for Taiwan traffic safety system
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Color mapping for different classes
        self.class_colors = {
            0: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),    # bicycle - green
            1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),    # bus - red
            2: ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),    # car - blue
            3: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),    # crosswalk - yellow
            4: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),    # greenlight - green
            5: ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),    # motorcycle - orange
            6: ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),    # pedestrian - magenta
            7: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),    # redlight - red
            8: ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),    # sidewalk - gray
            9: ColorRGBA(r=0.0, g=0.5, b=0.5, a=0.8),    # truck - teal
            10: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),   # yellowlight - yellow
        }
        
        # Subscribe to detection messages
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/camera/detections',
            self.detection_callback,
            10
        )
        
        # Publish marker array
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/camera/detections/markers',
            10
        )
        
        self.get_logger().info('🔍 Detection to Markers Converter Started')
        self.get_logger().info('📡 Converting /camera/detections to /camera/detections/markers')
    
    def detection_callback(self, msg):
        """Convert Detection2DArray to MarkerArray"""
        marker_array = MarkerArray()
        
        for i, detection in enumerate(msg.detections):
            if detection.results:
                class_id = int(detection.results[0].hypothesis.class_id)
                confidence = detection.results[0].hypothesis.score
                
                # Create bounding box marker
                marker = Marker()
                marker.header = msg.header
                marker.ns = "yolov8_detections"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Set position (convert from 2D to 3D)
                marker.pose.position.x = detection.bbox.center.position.x / 100.0  # Scale down
                marker.pose.position.y = detection.bbox.center.position.y / 100.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Set size
                marker.scale.x = detection.bbox.size_x / 100.0
                marker.scale.y = detection.bbox.size_y / 100.0
                marker.scale.z = 0.1  # Small height for 2D detections
                
                # Set color based on class
                marker.color = self.class_colors.get(class_id, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8))
                
                # Set text for class name and confidence
                class_name = self.class_names.get(class_id, f'Unknown({class_id})')
                marker.text = f"{class_name} ({confidence:.2f})"
                
                marker_array.markers.append(marker)
                
                # Create text marker for label
                text_marker = Marker()
                text_marker.header = msg.header
                text_marker.ns = "yolov8_labels"
                text_marker.id = i + 1000  # Different ID range
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = detection.bbox.center.position.x / 100.0
                text_marker.pose.position.y = detection.bbox.center.position.y / 100.0
                text_marker.pose.position.z = 0.2  # Slightly above the box
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.x = 0.1
                text_marker.scale.y = 0.1
                text_marker.scale.z = 0.1
                
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text
                text_marker.text = f"{class_name}\n{confidence:.2f}"
                
                marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
        
        if marker_array.markers:
            self.get_logger().debug(f'📊 Published {len(marker_array.markers)} markers')

def main():
    rclpy.init()
    
    converter = DetectionToMarkers()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        print("\n🛑 Detection to Markers Converter stopped by user")
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 