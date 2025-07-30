#!/usr/bin/env python3
"""
Decision Engine - Central Safety Coordinator
Updated for 2-Priority System: Vehicles > Traffic Lights
Removed Taiwan-specific crossing analysis (simplified system)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from enum import Enum

class CrossingDecision(Enum):
    SAFE_TO_CROSS = "SAFE_TO_CROSS"
    WAIT_FOR_VEHICLES = "WAIT_FOR_VEHICLES"
    WAIT_FOR_TRAFFIC_LIGHT = "WAIT_FOR_TRAFFIC_LIGHT"
    MANUAL = "MANUAL"

class DecisionEngine(Node):
    def __init__(self):
        super().__init__('decision_engine')
        
        # NEW: 2-Priority Safety System Parameters
        self.declare_parameter('vehicle_override_enabled', True)    # Priority 1 absolute override
        self.declare_parameter('traffic_light_weight', 0.8)        # Priority 2 weight
        self.declare_parameter('high_confidence_threshold', 0.8)
        self.declare_parameter('medium_confidence_threshold', 0.6)
        self.declare_parameter('low_confidence_threshold', 0.4)
        
        # Load parameters
        self.vehicle_override_enabled = self.get_parameter('vehicle_override_enabled').value
        self.traffic_light_weight = self.get_parameter('traffic_light_weight').value
        self.high_confidence_threshold = self.get_parameter('high_confidence_threshold').value
        self.medium_confidence_threshold = self.get_parameter('medium_confidence_threshold').value
        self.low_confidence_threshold = self.get_parameter('low_confidence_threshold').value
        
        # State tracking
        self.vehicle_threat_detected = False
        self.traffic_light_state = "unknown"
        self.traffic_light_confidence = 0.0
        
        # Subscriptions - NEW: Simplified 2-priority system
        # Priority 1: Vehicle threats (absolute override)
        self.vehicle_threat_sub = self.create_subscription(
            Bool, '/traffic_safety/immediate_crossing_danger',
            self.vehicle_threat_callback, 10)
        
        # Priority 2: Traffic lights
        self.traffic_light_sub = self.create_subscription(
            String, '/traffic_light_state',
            self.traffic_light_callback, 10)
        
        self.traffic_confidence_sub = self.create_subscription(
            Float32, '/traffic_light_confidence',
            self.traffic_confidence_callback, 10)
        
        # Publishers
        self.decision_pub = self.create_publisher(
            String, '/traffic_safety/crossing_decision', 10)
        
        self.reasoning_pub = self.create_publisher(
            String, '/traffic_safety/decision_reasoning', 10)
        
        # Timer for decision making (10 Hz)
        self.create_timer(0.1, self.make_crossing_decision)
        
        self.get_logger().info('🧠 Decision Engine initialized - 2-Priority System')
        self.get_logger().info('Priority 1: Vehicles (ABSOLUTE) | Priority 2: Traffic Lights')
    
    # Priority 1 callbacks
    def vehicle_threat_callback(self, msg: Bool):
        self.vehicle_threat_detected = msg.data
    
    # Priority 2 callbacks  
    def traffic_light_callback(self, msg: String):
        self.traffic_light_state = msg.data
    
    def traffic_confidence_callback(self, msg: Float32):
        self.traffic_light_confidence = msg.data
    
    def make_crossing_decision(self):
        """Implement 2-priority hierarchy decision logic"""
        
        # PRIORITY 1: Vehicle Threat Analysis (Absolute Override)
        if self.vehicle_threat_detected:
            decision = CrossingDecision.WAIT_FOR_VEHICLES
            reasoning = "PRIORITY 1: Immediate vehicle threat detected - Wait for vehicles to clear"
            confidence = 0.95
            
        # PRIORITY 2: Traffic Light Analysis
        elif self.traffic_light_state == "red":
            decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
            reasoning = f"PRIORITY 2: RED LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Wait for green light"
            confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        elif self.traffic_light_state == "green":
            decision = CrossingDecision.SAFE_TO_CROSS
            reasoning = f"PRIORITY 2: GREEN LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Safe to cross"
            confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        elif self.traffic_light_state == "yellow":
            decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
            reasoning = f"PRIORITY 2: YELLOW LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Proceed with caution"
            confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        # Default: No clear signal
        else:
            decision = CrossingDecision.MANUAL
            reasoning = "No clear traffic signal detected - Manual verification required"
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
        if decision == CrossingDecision.MANUAL:
            self.get_logger().warn(f'🚨 {decision.value}: {reasoning}')
        elif decision == CrossingDecision.SAFE_TO_CROSS:
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
