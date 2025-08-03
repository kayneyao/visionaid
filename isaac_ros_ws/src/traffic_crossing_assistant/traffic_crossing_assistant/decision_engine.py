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
        
        # Temporal stability parameters
        self.declare_parameter('vehicle_threat_memory_duration', 3.0)  # Remember vehicle threats for 3 seconds
        self.declare_parameter('traffic_light_memory_duration', 5.0)   # Remember traffic lights for 5 seconds
        self.declare_parameter('signal_phase_change_memory', 3.0)      # Remember signal phase changes for 3 seconds (reduced for safety)
        self.declare_parameter('phase_change_confidence_threshold', 0.6)  # Minimum confidence for phase change memory
        self.declare_parameter('decision_stability_duration', 2.0)     # Keep decisions stable for 2 seconds
        
        self.vehicle_threat_memory_duration = self.get_parameter('vehicle_threat_memory_duration').value
        self.traffic_light_memory_duration = self.get_parameter('traffic_light_memory_duration').value
        self.signal_phase_change_memory = self.get_parameter('signal_phase_change_memory').value
        self.phase_change_confidence_threshold = self.get_parameter('phase_change_confidence_threshold').value
        self.decision_stability_duration = self.get_parameter('decision_stability_duration').value
        
        # Temporal tracking
        self.last_vehicle_threat_time = 0.0
        self.last_traffic_light_time = 0.0
        self.last_signal_phase_change_time = 0.0
        self.last_decision_change_time = 0.0
        self.current_decision = CrossingDecision.MANUAL
        self.current_reasoning = "Initializing..."
        self.current_confidence = 0.1
        
        # Traffic light memory
        self.last_traffic_light_state = "unknown"
        self.last_traffic_light_confidence = 0.0
        self.previous_traffic_light_state = "unknown"  # Track previous state for phase change detection
        
        # Subscriptions - NEW: Simplified 2-priority system
        # Priority 1: Vehicle threats (absolute override)
        self.vehicle_threat_sub = self.create_subscription(
            Bool, '/immediate_crossing_danger',
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
        import time
        current_time = time.time()
        
        if msg.data:  # Vehicle threat detected
            self.vehicle_threat_detected = True
            self.last_vehicle_threat_time = current_time
            self.get_logger().info(f'🚨 VEHICLE THREAT RECEIVED: {msg.data}')
        else:  # No vehicle threat
            # Only clear threat if enough time has passed
            if current_time - self.last_vehicle_threat_time > self.vehicle_threat_memory_duration:
                self.vehicle_threat_detected = False
                self.get_logger().info(f'✅ Vehicle threat cleared after {self.vehicle_threat_memory_duration}s')
    
    # Priority 2 callbacks  
    def traffic_light_callback(self, msg: String):
        import time
        current_time = time.time()
        
        if msg.data != "unknown":  # Valid traffic light detected
            # Check for signal phase change (only for high confidence detections)
            if (self.previous_traffic_light_state != "unknown" and 
                self.previous_traffic_light_state != msg.data and
                self.traffic_light_confidence >= self.phase_change_confidence_threshold):
                # Signal phase change detected with high confidence!
                self.last_signal_phase_change_time = current_time
                self.get_logger().info(f'🔄 Signal phase change: {self.previous_traffic_light_state} → {msg.data} (conf: {self.traffic_light_confidence:.3f})')
            elif (self.previous_traffic_light_state != "unknown" and 
                  self.previous_traffic_light_state != msg.data):
                # Signal phase change detected but low confidence - log warning
                self.get_logger().warn(f'⚠️ Low confidence phase change ignored: {self.previous_traffic_light_state} → {msg.data} (conf: {self.traffic_light_confidence:.3f} < {self.phase_change_confidence_threshold})')
            
            self.traffic_light_state = msg.data
            self.last_traffic_light_time = current_time
            self.last_traffic_light_state = msg.data
            self.previous_traffic_light_state = msg.data  # Update previous state
        else:  # No traffic light detected
            # Only clear if enough time has passed (use longer memory for phase changes)
            time_since_phase_change = current_time - self.last_signal_phase_change_time
            time_since_last_light = current_time - self.last_traffic_light_time
            
            # Use longer memory if this was a recent phase change
            if time_since_phase_change < self.signal_phase_change_memory:
                # Keep the last known state for phase change memory duration
                pass  # Don't clear the state
            elif time_since_last_light > self.traffic_light_memory_duration:
                # Clear only if it's been long enough since last detection
                self.traffic_light_state = "unknown"
                self.get_logger().info(f'⚠️ Traffic light cleared after {self.traffic_light_memory_duration}s')
        
        # Update current state
        if msg.data != "unknown":
            self.traffic_light_state = msg.data
    
    def traffic_confidence_callback(self, msg: Float32):
        import time
        current_time = time.time()
        
        if msg.data > 0.0:  # Valid confidence
            self.traffic_light_confidence = msg.data
            self.last_traffic_light_confidence = msg.data
        else:  # Zero confidence
            # Only clear if enough time has passed
            if current_time - self.last_traffic_light_time > self.traffic_light_memory_duration:
                self.traffic_light_confidence = 0.0
                self.get_logger().info(f'⚠️ Traffic light confidence cleared after {self.traffic_light_memory_duration}s')
            # Otherwise keep the last known confidence
    
    def make_crossing_decision(self):
        """Implement 2-priority hierarchy decision logic with temporal stability"""
        import time
        current_time = time.time()
        
        # Calculate new decision
        new_decision = None
        new_reasoning = ""
        new_confidence = 0.0
        
        # PRIORITY 1: Vehicle Threat Analysis (Absolute Override)
        if self.vehicle_threat_detected:
            new_decision = CrossingDecision.WAIT_FOR_VEHICLES
            new_reasoning = "PRIORITY 1: Immediate vehicle threat detected - Wait for vehicles to clear"
            new_confidence = 0.95
            
        # PRIORITY 2: Traffic Light Analysis (with memory)
        current_time = time.time()
        time_since_last_light = current_time - self.last_traffic_light_time
        time_since_phase_change = current_time - self.last_signal_phase_change_time
        
        # Use remembered traffic light if recent enough (with phase change consideration)
        if (self.last_traffic_light_state != "unknown" and 
            (time_since_last_light <= self.traffic_light_memory_duration or 
             time_since_phase_change <= self.signal_phase_change_memory)):
            
            if self.last_traffic_light_state == "red":
                new_decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
                if time_since_phase_change <= self.signal_phase_change_memory:
                    new_reasoning = f"PRIORITY 2: RED LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Wait for green light (phase change remembered for {time_since_phase_change:.1f}s)"
                else:
                    new_reasoning = f"PRIORITY 2: RED LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Wait for green light (remembered for {time_since_last_light:.1f}s)"
                new_confidence = self.last_traffic_light_confidence * self.traffic_light_weight
                
            elif self.last_traffic_light_state == "green":
                new_decision = CrossingDecision.SAFE_TO_CROSS
                if time_since_phase_change <= self.signal_phase_change_memory:
                    new_reasoning = f"PRIORITY 2: GREEN LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Safe to cross (phase change remembered for {time_since_phase_change:.1f}s)"
                else:
                    new_reasoning = f"PRIORITY 2: GREEN LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Safe to cross (remembered for {time_since_last_light:.1f}s)"
                new_confidence = self.last_traffic_light_confidence * self.traffic_light_weight
                
            elif self.last_traffic_light_state == "yellow":
                new_decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
                if time_since_phase_change <= self.signal_phase_change_memory:
                    new_reasoning = f"PRIORITY 2: YELLOW LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Proceed with caution (phase change remembered for {time_since_phase_change:.1f}s)"
                else:
                    new_reasoning = f"PRIORITY 2: YELLOW LIGHT detected (conf: {self.last_traffic_light_confidence:.3f}) - Proceed with caution (remembered for {time_since_last_light:.1f}s)"
                new_confidence = self.last_traffic_light_confidence * self.traffic_light_weight
                
        # Use current traffic light state if available
        elif self.traffic_light_state == "red":
            new_decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
            new_reasoning = f"PRIORITY 2: RED LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Wait for green light"
            new_confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        elif self.traffic_light_state == "green":
            new_decision = CrossingDecision.SAFE_TO_CROSS
            new_reasoning = f"PRIORITY 2: GREEN LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Safe to cross"
            new_confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        elif self.traffic_light_state == "yellow":
            new_decision = CrossingDecision.WAIT_FOR_TRAFFIC_LIGHT
            new_reasoning = f"PRIORITY 2: YELLOW LIGHT detected (conf: {self.traffic_light_confidence:.3f}) - Proceed with caution"
            new_confidence = self.traffic_light_confidence * self.traffic_light_weight
            
        # Default: No clear signal
        else:
            new_decision = CrossingDecision.MANUAL
            new_reasoning = "No clear traffic signal detected - Manual verification required"
            new_confidence = 0.1
        
        # Apply temporal stability: only change decision if enough time has passed
        if (new_decision != self.current_decision and 
            current_time - self.last_decision_change_time > self.decision_stability_duration):
            
            # Update current decision
            self.current_decision = new_decision
            self.current_reasoning = new_reasoning
            self.current_confidence = new_confidence
            self.last_decision_change_time = current_time
            
            # Log decision change
            self.get_logger().info(f'🔄 Decision changed: {self.current_decision.value} - {self.current_reasoning}')
        
        # Publish current decision (which may be the stable previous decision)
        self.publish_decision(self.current_decision, self.current_reasoning, self.current_confidence)
    
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
