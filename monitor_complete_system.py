#!/usr/bin/env python3
"""
Complete Traffic Crossing Assistant System Monitor
Monitors all components and shows decision reasoning
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import defaultdict, deque
from datetime import datetime

class CompleteSystemMonitor(Node):
    def __init__(self):
        super().__init__('complete_system_monitor')
        
        self.bridge = CvBridge()
        
        # System status tracking
        self.system_status = {
            'yolov8_detection': False,
            'ego_motion_compensator': False,
            'vehicle_analyzer': False,
            'crosswalk_analyzer': False,
            'traffic_light_analyzer': False,
            'decision_engine': False,
            'audio_system': False,
            'multimodal_coordinator': False
        }
        
        # Decision tracking
        self.latest_decision = "No decision yet"
        self.latest_reasoning = "No reasoning available"
        self.decision_history = deque(maxlen=10)
        
        # Traffic light status
        self.traffic_light_state = "Unknown"
        self.traffic_light_confidence = 0.0
        
        # Vehicle threat status
        self.immediate_danger = False
        self.time_to_collision = 0.0
        self.vehicle_threat_status = "No threat"
        
        # Crosswalk status
        self.crosswalk_detected = False
        self.crosswalk_confidence = 0.0
        
        # Motion compensation status
        self.motion_compensation_quality = 0.0
        self.excessive_motion_detected = False
        
        # Statistics
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_stats = deque(maxlen=30)
        
        # Detection tracking
        self.detection_stats = defaultdict(int)  # Current frame detections
        self.latest_confidences = {}
        self.last_detection_time = 0
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Vehicle analyzer debug messages
        self.vehicle_debug_messages = []
        
        # Subscribe to all system topics
        self.setup_subscriptions()
        
        # Display timer
        self.create_timer(2.0, self.display_system_status)
        
        self.get_logger().info('🔍 Complete Traffic Crossing Assistant System Monitor Started')
        self.get_logger().info('📊 Monitoring all system components and decision reasoning')
    
    def setup_subscriptions(self):
        """Setup subscriptions to all system topics"""
        
        # Detection system
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections', self.detection_callback, 10)
        
        # Decision system
        self.decision_sub = self.create_subscription(
            String, '/traffic_safety/crossing_decision', self.decision_callback, 10)
        
        self.reasoning_sub = self.create_subscription(
            String, '/traffic_safety/decision_reasoning', self.reasoning_callback, 10)
        
        # Traffic light analysis
        self.traffic_light_state_sub = self.create_subscription(
            String, '/traffic_safety/traffic_light_state', self.traffic_light_state_callback, 10)
        
        self.traffic_light_conf_sub = self.create_subscription(
            Float32, '/traffic_safety/traffic_light_confidence', self.traffic_light_conf_callback, 10)
        
        # Vehicle analysis
        self.immediate_danger_sub = self.create_subscription(
            Bool, '/traffic_safety/immediate_crossing_danger', self.immediate_danger_callback, 10)
        
        self.ttc_sub = self.create_subscription(
            Float32, '/traffic_safety/time_to_collision', self.ttc_callback, 10)
        
        self.vehicle_threat_sub = self.create_subscription(
            String, '/traffic_safety/vehicle_threat_status', self.vehicle_threat_callback, 10)
        
        # Crosswalk analysis
        self.crosswalk_detected_sub = self.create_subscription(
            Bool, '/traffic_safety/crosswalk_detected', self.crosswalk_detected_callback, 10)
        
        self.crosswalk_conf_sub = self.create_subscription(
            Float32, '/traffic_safety/crosswalk_confidence', self.crosswalk_conf_callback, 10)
        
        # Motion compensation
        self.motion_quality_sub = self.create_subscription(
            Float32, '/traffic_safety/motion_compensation_quality', self.motion_quality_callback, 10)
        
        self.excessive_motion_sub = self.create_subscription(
            Bool, '/traffic_safety/excessive_motion_detected', self.excessive_motion_callback, 10)
        
        # Audio system
        self.audio_message_sub = self.create_subscription(
            String, '/traffic_safety/audio_message', self.audio_message_callback, 10)
        
        # VLM system
        self.vlm_description_sub = self.create_subscription(
            String, '/traffic_safety/vlm_scene_description', self.vlm_description_callback, 10)
        
        self.safety_reasoning_sub = self.create_subscription(
            String, '/traffic_safety/safety_reasoning_explanation', self.safety_reasoning_callback, 10)
        
        # Vehicle analyzer debug messages
        self.vehicle_debug_sub = self.create_subscription(
            String, '/vehicle_analyzer_debug', self.vehicle_debug_callback, 10)
    
    def detection_callback(self, msg):
        """Track detection system activity"""
        self.system_status['yolov8_detection'] = True
        self.frame_count += 1
        
        # Calculate FPS
        current_time = time.time()
        if self.last_time > 0:
            fps = 1.0 / (current_time - self.last_time)
            self.fps_stats.append(fps)
        self.last_time = current_time
        
        # Track current frame detections by class
        detections_this_frame = 0
        
        # Clear stats only if we have new detections (to avoid stale data)
        if msg.detections:
            self.detection_stats.clear()
        
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Update current frame statistics
                self.detection_stats[class_id] += 1
                self.latest_confidences[class_id] = confidence
                detections_this_frame += 1
        
        # Debug: Log detection count (removed to reduce spam)
        # if detections_this_frame > 0:
        #     self.get_logger().info(f'📊 Monitor: {detections_this_frame} detections in frame')
        
        # Update last detection time
        self.last_detection_time = current_time
    
    def decision_callback(self, msg):
        """Track crossing decisions"""
        self.system_status['decision_engine'] = True
        self.latest_decision = msg.data
        self.decision_history.append({
            'decision': msg.data,
            'timestamp': time.time()
        })
    
    def reasoning_callback(self, msg):
        """Track decision reasoning"""
        self.latest_reasoning = msg.data
    
    def traffic_light_state_callback(self, msg):
        """Track traffic light state"""
        self.system_status['traffic_light_analyzer'] = True
        self.traffic_light_state = msg.data
    
    def traffic_light_conf_callback(self, msg):
        """Track traffic light confidence"""
        self.traffic_light_confidence = msg.data
    
    def immediate_danger_callback(self, msg):
        """Track immediate danger status"""
        self.system_status['vehicle_analyzer'] = True
        self.immediate_danger = msg.data
    
    def ttc_callback(self, msg):
        """Track time to collision"""
        self.system_status['vehicle_analyzer'] = True
        self.time_to_collision = msg.data
    
    def vehicle_threat_callback(self, msg):
        """Track vehicle threat status"""
        self.system_status['vehicle_analyzer'] = True
        self.vehicle_threat_status = msg.data
    
    def crosswalk_detected_callback(self, msg):
        """Track crosswalk detection"""
        self.system_status['crosswalk_analyzer'] = True
        self.crosswalk_detected = msg.data
    
    def crosswalk_conf_callback(self, msg):
        """Track crosswalk confidence"""
        self.crosswalk_confidence = msg.data
    
    def motion_quality_callback(self, msg):
        """Track motion compensation quality"""
        self.system_status['ego_motion_compensator'] = True
        self.motion_compensation_quality = msg.data
    
    def excessive_motion_callback(self, msg):
        """Track excessive motion detection"""
        self.excessive_motion_detected = msg.data
    
    def audio_message_callback(self, msg):
        """Track audio system activity"""
        self.system_status['audio_system'] = True
    
    def vlm_description_callback(self, msg):
        """Track VLM scene description"""
        self.system_status['multimodal_coordinator'] = True
    
    def safety_reasoning_callback(self, msg):
        """Track safety reasoning"""
        pass  # Already handled by reasoning callback
    
    def vehicle_debug_callback(self, msg):
        """Track vehicle analyzer debug messages"""
        self.vehicle_debug_messages.append(msg.data)
        # Keep only last 5 messages to avoid spam
        if len(self.vehicle_debug_messages) > 5:
            self.vehicle_debug_messages.pop(0)
    

    
    def display_system_status(self):
        """Display comprehensive system status"""
        avg_fps = np.mean(self.fps_stats) if self.fps_stats else 0
        
        print("\n" + "="*80)
        print("🚦 COMPLETE TRAFFIC CROSSING ASSISTANT SYSTEM STATUS")
        print("="*80)
        
        # System component status
        print("🔧 SYSTEM COMPONENTS:")
        for component, status in self.system_status.items():
            status_icon = "✅" if status else "❌"
            print(f"   {status_icon} {component.replace('_', ' ').title()}")
        
        # Performance
        print(f"\n⚡ PERFORMANCE: {avg_fps:.1f} FPS, {self.frame_count} frames processed")
        
        # Current frame detections
        current_time = time.time()
        time_since_last_detection = current_time - self.last_detection_time
        
        if self.detection_stats and time_since_last_detection < 2.0:
            print(f"\n🎯 CURRENT YOLOV8 DETECTIONS:")
            sorted_stats = sorted(self.detection_stats.items(), key=lambda x: x[1], reverse=True)
            for class_id, count in sorted_stats:
                class_name = self.class_names.get(int(class_id), f'Unknown({class_id})')
                confidence = self.latest_confidences.get(class_id, 0.0)
                print(f"   {class_name}: {count} objects (conf: {confidence:.3f})")
            print(f"   (Last detection: {time_since_last_detection:.1f}s ago)")
        else:
            print(f"\n🎯 CURRENT YOLOV8 DETECTIONS: No objects detected in current frame")
            if time_since_last_detection < 10.0:
                print(f"   (Last detection: {time_since_last_detection:.1f}s ago)")
        
        # Current decision and reasoning
        print(f"\n🎯 CURRENT DECISION: {self.latest_decision}")
        print(f"🧠 DECISION REASONING: {self.latest_reasoning}")
        
        # Traffic light analysis
        print(f"\n🚦 TRAFFIC LIGHT ANALYSIS:")
        print(f"   State: {self.traffic_light_state}")
        print(f"   Confidence: {self.traffic_light_confidence:.3f}")
        
        # Vehicle threat analysis
        print(f"\n🚗 VEHICLE THREAT ANALYSIS:")
        print(f"   Immediate Danger: {'🚨 YES' if self.immediate_danger else '✅ No'}")
        print(f"   Time to Collision: {self.time_to_collision:.2f}s")
        print(f"   Threat Status: {self.vehicle_threat_status}")
        
        # Vehicle analyzer debug messages
        if self.vehicle_debug_messages:
            print(f"\n🔍 VEHICLE ANALYZER DEBUG:")
            for msg in self.vehicle_debug_messages:
                print(f"   {msg}")
        
        # Crosswalk analysis
        print(f"\n🚶 CROSSWALK ANALYSIS:")
        print(f"   Detected: {'✅ Yes' if self.crosswalk_detected else '❌ No'}")
        print(f"   Confidence: {self.crosswalk_confidence:.3f}")
        
        # Motion compensation
        print(f"\n📹 MOTION COMPENSATION:")
        print(f"   Quality: {self.motion_compensation_quality:.3f}")
        print(f"   Excessive Motion: {'⚠️ Yes' if self.excessive_motion_detected else '✅ No'}")
        
        # Recent decision history
        if self.decision_history:
            print(f"\n📋 RECENT DECISIONS:")
            for i, decision_info in enumerate(list(self.decision_history)[-3:]):
                time_ago = time.time() - decision_info['timestamp']
                print(f"   {i+1}. {decision_info['decision']} ({time_ago:.1f}s ago)")
        
        print("="*80)
        print("Press Ctrl+C to stop monitoring")

def main():
    rclpy.init()
    monitor = CompleteSystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 Complete system monitoring stopped by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 