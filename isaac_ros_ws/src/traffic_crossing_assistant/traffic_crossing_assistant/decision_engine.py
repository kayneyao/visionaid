#!/usr/bin/env python3
"""
Decision Engine - Main Coordinator for 3-Priority Hierarchy
Implements: Vehicles → Taiwan Crossing Path → Traffic Lights
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from enum import Enum

class CrossingDecision(Enum):
    DONT_CROSS = "DONT_CROSS"
    CROSS = "CROSS"
    MANUAL = "MANUAL"

class DecisionEngine(Node):
    def __init__(self):
        super().__init__('decision_engine')
        
        # State tracking for 3-priority system
        self.immediate_danger = False           # Priority 1
        self.crossing_path_confirmed = False    # Priority 2
        self.spatial_confidence = 0.0          # Priority 2 confidence
        self.traffic_light_state = "unknown"   # Priority 3
        self.signal_confidence = 0.0           # Priority 3 confidence
        
        # Priority weights based on your research
        self.priority_weights = {
            'vehicle_authority': 1.0,      # Absolute override
            'crossing_authority': 0.4,     # Your 85.5% mAP50 innovation
            'traffic_authority': 0.3       # Conservative signal analysis
        }
        
        # Subscriptions - Priority 1: Vehicle Movement
        self.danger_sub = self.create_subscription(
            Bool, '/immediate_crossing_danger',
            self.immediate_danger_callback, 10)
        
        # Subscriptions - Priority 2: Taiwan Crossing Path
        self.crossing_sub = self.create_subscription(
            Bool, '/crossing_path_confirmed',
            self.crossing_path_callback, 10)
        
        self.spatial_conf_sub = self.create_subscription(
            Float32, '/spatial_classification_confidence',
            self.spatial_confidence_callback, 10)
        
        # Subscriptions - Priority 3: Traffic Lights
        self.traffic_sub = self.create_subscription(
            String, '/traffic_light_state',
            self.traffic_light_callback, 10)
        
        self.signal_conf_sub = self.create_subscription(
            Float32, '/traffic_light_confidence',
            self.signal_confidence_callback, 10)
        
        # Publishers
        self.decision_pub = self.create_publisher(
            String, '/crossing_decision', 10)
        
        self.reasoning_pub = self.create_publisher(
            String, '/decision_reasoning', 10)
        
        # Timer for decision making (10 Hz)
        self.create_timer(0.1, self.make_crossing_decision)
        
        self.get_logger().info('🧠 Decision Engine initialized')
        self.get_logger().info('Priority: Vehicles → Taiwan Crossing (85.5% mAP50) → Traffic Lights')
    
    # Priority 1 callbacks
    def immediate_danger_callback(self, msg: Bool):
        self.immediate_danger = msg.data
    
    # Priority 2 callbacks  
    def crossing_path_callback(self, msg: Bool):
        self.crossing_path_confirmed = msg.data
    
    def spatial_confidence_callback(self, msg: Float32):
        self.spatial_confidence = msg.data
    
    # Priority 3 callbacks
    def traffic_light_callback(self, msg: String):
        self.traffic_light_state = msg.data
    
    def signal_confidence_callback(self, msg: Float32):
        self.signal_confidence = msg.data
    
    def make_crossing_decision(self):
        """Implement 3-priority hierarchy decision logic"""
        
        # PRIORITY 1: Vehicle Movement Analysis (Absolute Override)
        if self.immediate_danger:
            decision = CrossingDecision.DONT_CROSS
            reasoning = "PRIORITY 1: Immediate vehicle danger detected - DO NOT CROSS"
            confidence = 0.95
            
        # PRIORITY 2: Taiwan Crossing Path Authority (Your 85.5% mAP50 Innovation)
        elif self.crossing_path_confirmed and self.spatial_confidence > 0.7:
            # Your Taiwan spatial classification detected - proceed to traffic analysis
            if self.traffic_light_state == "red":
                decision = CrossingDecision.DONT_CROSS
                reasoning = f"PRIORITY 2+3: Taiwan crossing confirmed (conf: {self.spatial_confidence:.3f}) + RED LIGHT detected"
                confidence = self.spatial_confidence * 0.4 + self.signal_confidence * 0.3
                
            elif self.traffic_light_state == "green":
                decision = CrossingDecision.CROSS
                reasoning = f"PRIORITY 2+3: Taiwan crossing confirmed (conf: {self.spatial_confidence:.3f}) + GREEN LIGHT - Proceed with caution"
                confidence = self.spatial_confidence * 0.4 + self.signal_confidence * 0.3
                
            else:  # No traffic signal
                decision = CrossingDecision.MANUAL
                reasoning = f"PRIORITY 2: Taiwan crossing confirmed (conf: {self.spatial_confidence:.3f}) - No traffic signal detected, manual verification required"
                confidence = self.spatial_confidence * 0.4
        
        # PRIORITY 3: Traffic Light Analysis Only (No crossing path confirmed)
        elif self.traffic_light_state == "red":
            decision = CrossingDecision.DONT_CROSS
            reasoning = f"PRIORITY 3: RED LIGHT detected (conf: {self.signal_confidence:.3f}) - DO NOT CROSS"
            confidence = self.signal_confidence * 0.3
            
        elif self.traffic_light_state == "green":
            decision = CrossingDecision.MANUAL
            reasoning = f"PRIORITY 3: GREEN LIGHT detected (conf: {self.signal_confidence:.3f}) but no crossing path confirmed - Manual verification required"
            confidence = self.signal_confidence * 0.3
        
        # Default: Insufficient information
        else:
            decision = CrossingDecision.MANUAL
            reasoning = "Insufficient information for safe crossing decision - Manual verification required"
            confidence = 0.1
        
        # Publish decision
        self.publish_decision(decision, reasoning, confidence)
    
    def publish_decision(self, decision, reasoning, confidence):
        """Publish crossing decision and reasoning"""
        
        # Publish decision
        decision_msg = String()
        decision_msg.data = f"{decision.value}|{confidence:.2f}"
        self.decision_pub.publish(decision_msg)
        
        # Publish reasoning
        reasoning_msg = String()
        reasoning_msg.data = reasoning
        self.reasoning_pub.publish(reasoning_msg)
        
        # Log decision
        if decision == CrossingDecision.DONT_CROSS:
            self.get_logger().warn(f'🚨 {decision.value}: {reasoning}')
        elif decision == CrossingDecision.CROSS:
            self.get_logger().info(f'✅ {decision.value}: {reasoning}')
        else:
            self.get_logger().info(f'⚠️ {decision.value}: {reasoning}')

def main():
    rclpy.init()
    node = DecisionEngine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
