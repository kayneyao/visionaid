#!/usr/bin/env python3
"""
Crosswalk Analyzer - Context Information Component
Simplified for 11-class model: Basic crosswalk detection only
No longer part of priority safety system (provides context information)
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Point

class CrosswalkAnalyzer(Node):
    def __init__(self):
        super().__init__('crosswalk_analyzer')
        
        # NEW: 11-class crosswalk mapping (simplified)
        self.crosswalk_classes = {
            3: 'crosswalk'  # Class 3 - Single crosswalk class
        }
        
        # Context detection threshold
        self.crosswalk_threshold = 0.6
        
        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',
            self.detection_callback, 10)
        
        # Publishers (now for context information only)
        self.crosswalk_detected_pub = self.create_publisher(
            Bool, '/crosswalk_detected', 10)
        
        self.crosswalk_confidence_pub = self.create_publisher(
            Float32, '/crosswalk_confidence', 10)
        
        self.crosswalk_status_pub = self.create_publisher(
            String, '/crosswalk_status', 10)
        
        self.get_logger().info('🚶 Crosswalk Analyzer initialized (Context Information)')
        self.get_logger().info('Monitoring crosswalk (Class 3) for situational awareness')
    
    def detection_callback(self, msg: Detection2DArray):
        """Process 11-class detections for crosswalk context information"""
        
        crosswalks = []
        
        for detection in msg.detections:
            if detection.results:
                class_id = int(detection.results[0].hypothesis.class_id)
                confidence = detection.results[0].hypothesis.score
                
                # Check for crosswalk detection
                if class_id == 3 and confidence >= self.crosswalk_threshold:
                    crosswalks.append({
                        'confidence': confidence,
                        'bbox': detection.bbox
                    })
        
        # Analyze crosswalk context
        self.analyze_crosswalk_context(crosswalks)
    
    def analyze_crosswalk_context(self, crosswalks):
        """Analyze crosswalk detections for context information"""
        
        if crosswalks:
            # Find best crosswalk detection
            best_crosswalk = max(crosswalks, key=lambda x: x['confidence'])
            
            # Publish context information
            detected_msg = Bool()
            detected_msg.data = True
            self.crosswalk_detected_pub.publish(detected_msg)
            
            confidence_msg = Float32()
            confidence_msg.data = best_crosswalk['confidence']
            self.crosswalk_confidence_pub.publish(confidence_msg)
            
            status_msg = String()
            status_msg.data = f"Crosswalk detected (conf: {best_crosswalk['confidence']:.3f})"
            self.crosswalk_status_pub.publish(status_msg)
            
            self.get_logger().info(f'🚶 Crosswalk detected (conf: {best_crosswalk["confidence"]:.3f})')
        
        else:
            # No crosswalk detected
            detected_msg = Bool()
            detected_msg.data = False
            self.crosswalk_detected_pub.publish(detected_msg)
            
            confidence_msg = Float32()
            confidence_msg.data = 0.0
            self.crosswalk_confidence_pub.publish(confidence_msg)
            
            status_msg = String()
            status_msg.data = "No crosswalk detected"
            self.crosswalk_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
