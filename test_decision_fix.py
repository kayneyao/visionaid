#!/usr/bin/env python3
"""
Test Decision Engine Fix - Publish test traffic light messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import time

class DecisionEngineTester(Node):
    def __init__(self):
        super().__init__('decision_engine_tester')
        
        # Publishers for testing
        self.traffic_light_pub = self.create_publisher(
            String, '/traffic_light_state', 10)
        
        self.traffic_confidence_pub = self.create_publisher(
            Float32, '/traffic_light_confidence', 10)
        
        self.vehicle_threat_pub = self.create_publisher(
            Bool, '/immediate_crossing_danger', 10)
        
        # Subscribe to decision output
        self.decision_sub = self.create_subscription(
            String, '/traffic_safety/crossing_decision',
            self.decision_callback, 10)
        
        self.reasoning_sub = self.create_subscription(
            String, '/traffic_safety/decision_reasoning',
            self.reasoning_callback, 10)
        
        # Test sequence
        self.test_sequence = [
            ('red', 0.85, False, 3),      # Red light, no vehicle threat
            ('green', 0.90, False, 3),    # Green light, no vehicle threat  
            ('unknown', 0.0, True, 3),    # Unknown light, vehicle threat
            ('red', 0.80, False, 3),      # Red light, no vehicle threat
        ]
        
        self.current_test = 0
        self.test_start_time = time.time()
        
        # Start testing
        self.create_timer(1.0, self.run_test_sequence)
        
        self.get_logger().info('🧪 Decision Engine Tester started')
        self.get_logger().info('Will test traffic light states and vehicle threats')
    
    def run_test_sequence(self):
        if self.current_test >= len(self.test_sequence):
            self.get_logger().info('✅ Test sequence completed')
            return
        
        current_time = time.time()
        test_duration = self.test_sequence[self.current_test][3]
        
        if current_time - self.test_start_time >= test_duration:
            # Move to next test
            self.current_test += 1
            self.test_start_time = current_time
            
            if self.current_test < len(self.test_sequence):
                self.get_logger().info(f'🔄 Moving to test {self.current_test + 1}')
        
        # Publish current test
        if self.current_test < len(self.test_sequence):
            light_state, confidence, vehicle_threat, _ = self.test_sequence[self.current_test]
            
            # Publish traffic light
            light_msg = String()
            light_msg.data = light_state
            self.traffic_light_pub.publish(light_msg)
            
            # Publish confidence
            conf_msg = Float32()
            conf_msg.data = confidence
            self.traffic_confidence_pub.publish(conf_msg)
            
            # Publish vehicle threat
            threat_msg = Bool()
            threat_msg.data = vehicle_threat
            self.vehicle_threat_pub.publish(threat_msg)
            
            self.get_logger().info(f'📤 Test {self.current_test + 1}: Light={light_state}, Conf={confidence:.2f}, Threat={vehicle_threat}')
    
    def decision_callback(self, msg):
        self.get_logger().info(f'🧠 DECISION RECEIVED: {msg.data}')
    
    def reasoning_callback(self, msg):
        self.get_logger().info(f'💭 REASONING: {msg.data}')

def main():
    rclpy.init()
    node = DecisionEngineTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Tester stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 