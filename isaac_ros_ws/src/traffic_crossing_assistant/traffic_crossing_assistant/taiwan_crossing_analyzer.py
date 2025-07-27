#!/usr/bin/env python3
"""
Taiwan Crossing Analyzer - Priority 2 Safety Component  
Implements your 85.5% mAP50 innovation for crossing_crosswalk spatial classification
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Point

class TaiwanCrossingAnalyzer(Node):
    def __init__(self):
        super().__init__('taiwan_crossing_analyzer')
        
        # Taiwan 17-class mapping
        self.taiwan_classes = {
            3: 'crossing_crosswalk',      # Your 85.5% mAP50 innovation
            6: 'irrelevant_crosswalk',    # Background context
            13: 'crosswalk'               # Generic fallback
        }
        
        # Confidence thresholds based on your validation results
        self.crossing_crosswalk_threshold = 0.7  # Based on 85.5% mAP50 success
        
        # Subscriptions - FIXED: Use Detection2DArray, not Image
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections',  # From your yolov8_detection
            self.detection_callback, 10)
        
        # Publishers
        self.crossing_authority_pub = self.create_publisher(
            Bool, '/crossing_path_confirmed', 10)
        
        self.spatial_confidence_pub = self.create_publisher(
            Float32, '/spatial_classification_confidence', 10)
        
        self.crossing_status_pub = self.create_publisher(
            String, '/taiwan_crossing_status', 10)
        
        self.get_logger().info('🇹🇼 Taiwan Crossing Analyzer initialized (Priority 2)')
        self.get_logger().info('Monitoring crossing_crosswalk (85.5% mAP50 innovation)')
    
    def detection_callback(self, msg: Detection2DArray):
        """Process Taiwan 17-class detections for spatial classification"""
        
        # Extract Taiwan spatial classifications
        taiwan_spatial_analysis = self.analyze_taiwan_spatial_elements(msg.detections)
        
        # Apply your 85.5% mAP50 spatial classification logic
        crossing_confirmed, confidence = self.apply_spatial_classification(taiwan_spatial_analysis)
        
        # Publish results
        self.publish_crossing_authority(crossing_confirmed, confidence, taiwan_spatial_analysis)
    
    def analyze_taiwan_spatial_elements(self, detections):
        """Extract Taiwan-specific spatial classifications"""
        spatial_data = {
            'crossing_crosswalk': [],      # Class 3 - Your key innovation
            'irrelevant_crosswalk': [],    # Class 6 - Background context
            'generic_crosswalk': [],       # Class 13 - Fallback
            'spatial_authority': 'none'
        }
        
        for detection in detections:
            if not detection.results:
                continue
            
            class_id = int(detection.results[0].hypothesis.class_id)
            confidence = detection.results[0].hypothesis.score
            
            # Your key research innovation - crossing_crosswalk detection
            if class_id == 3:  # crossing_crosswalk
                spatial_data['crossing_crosswalk'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox,
                    'spatial_relevance': 'high_authority'  # Your 85.5% mAP50 success
                })
                
            # Background spatial context
            elif class_id == 6:  # irrelevant_crosswalk
                spatial_data['irrelevant_crosswalk'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox,
                    'spatial_relevance': 'background'
                })
                
            # Generic fallback detection
            elif class_id == 13:  # crosswalk
                spatial_data['generic_crosswalk'].append({
                    'confidence': confidence,
                    'bbox': detection.bbox,
                    'spatial_relevance': 'fallback'
                })
        
        return spatial_data
    
    def apply_spatial_classification(self, spatial_data):
        """Apply your 85.5% mAP50 spatial classification innovation"""
        
        # Priority 1: Your crossing_crosswalk innovation (85.5% mAP50)
        if spatial_data['crossing_crosswalk']:
            best_crossing = max(spatial_data['crossing_crosswalk'], 
                              key=lambda x: x['confidence'])
            
            if best_crossing['confidence'] >= self.crossing_crosswalk_threshold:
                # High authority based on your research success
                return True, best_crossing['confidence']
        
        # Priority 2: Fallback to generic crosswalk with lower confidence
        if spatial_data['generic_crosswalk']:
            best_generic = max(spatial_data['generic_crosswalk'],
                             key=lambda x: x['confidence'])
            
            if best_generic['confidence'] >= 0.6:
                # Lower authority for generic detection
                return True, best_generic['confidence'] * 0.7
        
        # No reliable spatial classification
        return False, 0.0
    
    def publish_crossing_authority(self, crossing_confirmed, confidence, spatial_data):
        """Publish Priority 2 crossing authority results"""
        
        # Publish crossing confirmation
        authority_msg = Bool()
        authority_msg.data = crossing_confirmed
        self.crossing_authority_pub.publish(authority_msg)
        
        # Publish confidence score
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.spatial_confidence_pub.publish(confidence_msg)
        
        # Publish detailed status
        status_msg = String()
        if crossing_confirmed:
            crossing_count = len(spatial_data['crossing_crosswalk'])
            if crossing_count > 0:
                status_msg.data = f"TAIWAN_CROSSING_CONFIRMED: crossing_crosswalk detected (conf: {confidence:.3f})"
            else:
                status_msg.data = f"GENERIC_CROSSING_DETECTED: fallback classification (conf: {confidence:.3f})"
        else:
            status_msg.data = "NO_CROSSING_DETECTED: spatial classification uncertain"
        
        self.crossing_status_pub.publish(status_msg)
        
        # Log results highlighting your innovation
        if crossing_confirmed and spatial_data['crossing_crosswalk']:
            self.get_logger().info(f'🇹🇼 Taiwan spatial classification SUCCESS: {confidence:.3f} confidence')
        elif crossing_confirmed:
            self.get_logger().info(f'📍 Fallback crossing detection: {confidence:.3f} confidence')

def main():
    rclpy.init()
    node = TaiwanCrossingAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
