#!/usr/bin/env python3
"""Test VLM integration with decision engine"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class VLMIntegrationTester(Node):
    def __init__(self):
        super().__init__('vlm_integration_tester')
        
        # Publishers to simulate decision engine
        self.decision_pub = self.create_publisher(String, '/crossing_decision', 10)
        self.voice_query_pub = self.create_publisher(String, '/user_voice_query', 10)
        
        # Subscribers to monitor VLM responses
        self.vlm_response_sub = self.create_subscription(
            String, '/safety_reasoning_explanation',
            self.vlm_response_callback, 10)
        
        self.get_logger().info('VLM Integration Tester ready')
    
    def test_dont_cross_explanation(self):
        """Test VLM explanation for DONT_CROSS decision"""
        self.get_logger().info('Testing DONT_CROSS VLM explanation...')
        
        decision_msg = String()
        decision_msg.data = "DONT_CROSS|0.85"
        self.decision_pub.publish(decision_msg)
    
    def vlm_response_callback(self, msg):
        self.get_logger().info(f'VLM Response: {msg.data}')
    
    def run_tests(self):
        """Run VLM integration tests"""
        time.sleep(2)
        self.test_dont_cross_explanation()

def main():
    rclpy.init()
    tester = VLMIntegrationTester()
    tester.run_tests()
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
