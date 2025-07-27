#!/usr/bin/env python3
"""
Traffic Light Analyzer - Priority 3 Safety Component
Conservative analysis using reliable red_light/green_light classes (14, 15)
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Float32
from enum import Enum

class TrafficLightState(Enum):
    RED = "red"
    GREEN = "green"
    UNKNOWN = "unknown"

class TrafficLightAnalyzer(Node):
    def __init__(self):
        super().__init__('traffic_light_analyzer')
        
        # Reliable traffic light classes (avoiding problematic Taiwan-specific variants)
        self.traffic_light_classes = {
            14: 'red_light',    # Class 14 - Original, more reliable
            15: 'green_light'   # Class 15 - Original, more reliable
        }
        
        # Conservative confidence thresholds
        self.red_light_threshold = 0.6   # Conservative for safety
        self.green_light_threshold = 0.6 # Conservative for safety
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        # Publishers
        self.traffic_signal_pub = self.create_publisher(
            String, '/traffic_light_state', 10)
        
        self.signal_confidence_pub = self.create_publisher(
            Float32, '/traffic_light_confidence', 10)
        
        self.get_logger().info('🚦 Traffic Light Analyzer initialized (Priority 3)')
        self.get_logger().info('Using reliable classes: red_light (14), green_light (15)')
    
    def detection_callback(self, msg: Detection2DArray):
        """Analyze traffic light detections with conservative approach"""
        
        # Extract traffic light detections
        light_detections = self.extract_traffic_light_detections(msg.detections)
        
        # Apply conservative traffic light analysis
        signal_state, confidence = self.analyze_traffic_signals(light_detections)
        
        # Publish results
        self.publish_traffic_signal_analysis(signal_state, confidence)
    
    def extract_traffic_light_detections(self, detections):
        """Extract reliable traffic light detections"""
        lights = {
            'red_lights': [],
            'green_lights': []
        }
        
        for detection in detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Only use reliable original traffic light classes
            if class_id == 14 and confidence >= self.red_light_threshold:  # red_light
                lights['red_lights'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox
                })
                
            elif class_id == 15 and confidence >= self.green_light_threshold:  # green_light
                lights['green_lights'].append({
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
        else:
            self.get_logger().info('⚪ No traffic signal detected - Manual verification required')

def main():
    rclpy.init()
    node = TrafficLightAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
