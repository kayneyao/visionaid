#!/usr/bin/env python3
"""
Traffic Light Analyzer - Priority 2 Safety Component
Updated for 11-class model with all 3 traffic light colors
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Float32
from enum import Enum

class TrafficLightState(Enum):
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    UNKNOWN = "unknown"

class TrafficLightAnalyzer(Node):
    def __init__(self):
        super().__init__('traffic_light_analyzer')
        
        # NEW: 11-class traffic light mapping (all 3 colors)
        self.traffic_light_classes = {
            4: 'greenlight',    # Class 4 - Go signal
            7: 'redlight',      # Class 7 - Stop signal
            10: 'yellowlight'   # Class 10 - Caution signal
        }
        
        # Confidence thresholds (matching YOLOv8 camera node exactly)
        self.red_light_threshold = 0.2      # redlight - very low for small objects
        self.green_light_threshold = 0.2    # greenlight - very low for small objects  
        self.yellow_light_threshold = 0.2   # yellowlight - very low for small objects
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        # Publishers
        self.traffic_signal_pub = self.create_publisher(
            String, '/traffic_light_state', 10)
        
        self.signal_confidence_pub = self.create_publisher(
            Float32, '/traffic_light_confidence', 10)
        
        self.get_logger().info('🚦 Traffic Light Analyzer initialized (Priority 2)')
        self.get_logger().info('Using all 3 colors: greenlight (4), redlight (7), yellowlight (10)')
    
    def detection_callback(self, msg: Detection2DArray):
        """Analyze traffic light detections with conservative approach"""
        
        # Debug: Log all detections
        for detection in msg.detections:
            if detection.results:
                class_id = int(detection.results[0].hypothesis.class_id)
                confidence = detection.results[0].hypothesis.score
                if class_id in [4, 7, 10]:  # Traffic light classes
                    self.get_logger().info(f'🚦 Traffic Light Detected: Class {class_id}, Confidence: {confidence:.3f}')
        
        # Extract traffic light detections
        light_detections = self.extract_traffic_light_detections(msg.detections)
        
        # Debug: Log extracted detections
        total_lights = len(light_detections['red_lights']) + len(light_detections['green_lights']) + len(light_detections['yellow_lights'])
        self.get_logger().info(f'🚦 Extracted {total_lights} traffic lights: R={len(light_detections["red_lights"])}, G={len(light_detections["green_lights"])}, Y={len(light_detections["yellow_lights"])}')
        
        # Apply conservative traffic light analysis
        signal_state, confidence = self.analyze_traffic_signals(light_detections)
        
        # Publish results
        self.publish_traffic_signal_analysis(signal_state, confidence)
    
    def extract_traffic_light_detections(self, detections):
        """Extract reliable traffic light detections"""
        lights = {
            'red_lights': [],
            'green_lights': [],
            'yellow_lights': []
        }
        
        for detection in detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # NEW: Use all 3 traffic light classes
            if class_id == 4 and confidence >= self.green_light_threshold:  # greenlight
                lights['green_lights'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox
                })
            elif class_id == 7 and confidence >= self.red_light_threshold:  # redlight
                lights['red_lights'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox
                })
            elif class_id == 10 and confidence >= self.yellow_light_threshold:  # yellowlight
                lights['yellow_lights'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox
                })
        
        return lights
    
    def analyze_traffic_signals(self, light_detections):
        """Conservative traffic signal analysis"""
        
        # Priority 1: Red light detection (safety first)
        if light_detections['red_lights']:
            best_red = max(light_detections['red_lights'], 
                          key=lambda x: x['confidence'])
            return TrafficLightState.RED, best_red['confidence']
        
        # Priority 2: Green light detection (proceed with caution)
        if light_detections['green_lights']:
            best_green = max(light_detections['green_lights'],
                           key=lambda x: x['confidence'])
            return TrafficLightState.GREEN, best_green['confidence']
        
        # Priority 3: Yellow light detection (proceed with caution)
        if light_detections['yellow_lights']:
            best_yellow = max(light_detections['yellow_lights'],
                              key=lambda x: x['confidence'])
            return TrafficLightState.YELLOW, best_yellow['confidence']
        
        # No reliable signal detected
        return TrafficLightState.UNKNOWN, 0.0
    
    def publish_traffic_signal_analysis(self, signal_state, confidence):
        """Publish Priority 3 traffic signal analysis"""
        
        # Publish signal state
        signal_msg = String()
        signal_msg.data = signal_state.value
        self.traffic_signal_pub.publish(signal_msg)
        
        # Publish confidence
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.signal_confidence_pub.publish(confidence_msg)
        
        # Log analysis results
        if signal_state == TrafficLightState.RED:
            self.get_logger().info(f'🔴 RED LIGHT detected (conf: {confidence:.3f}) - DO NOT CROSS')
        elif signal_state == TrafficLightState.GREEN:
            self.get_logger().info(f'🟢 GREEN LIGHT detected (conf: {confidence:.3f}) - Proceed with caution')
        elif signal_state == TrafficLightState.YELLOW:
            self.get_logger().info(f'🟡 YELLOW LIGHT detected (conf: {confidence:.3f}) - Proceed with caution')
        else:
            self.get_logger().info('⚪ No traffic signal detected - Manual verification required')

def main():
    rclpy.init()
    node = TrafficLightAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
