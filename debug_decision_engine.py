#!/usr/bin/env python3
"""
Debug Decision Engine - Monitor traffic light topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import time

class DecisionEngineDebugger(Node):
    def __init__(self):
        super().__init__('decision_engine_debugger')
        
        # Subscribe to all relevant topics
        self.traffic_light_state_sub = self.create_subscription(
            String, '/traffic_light_state',
            self.traffic_light_state_callback, 10)
        
        self.traffic_light_state_remapped_sub = self.create_subscription(
            String, '/traffic_safety/traffic_light_state',
            self.traffic_light_state_remapped_callback, 10)
        
        self.traffic_light_confidence_sub = self.create_subscription(
            Float32, '/traffic_light_confidence',
            self.traffic_light_confidence_callback, 10)
        
        self.traffic_light_confidence_remapped_sub = self.create_subscription(
            Float32, '/traffic_safety/traffic_light_confidence',
            self.traffic_light_confidence_remapped_callback, 10)
        
        self.vehicle_threat_sub = self.create_subscription(
            Bool, '/immediate_crossing_danger',
            self.vehicle_threat_callback, 10)
        
        self.vehicle_threat_remapped_sub = self.create_subscription(
            Bool, '/traffic_safety/immediate_crossing_danger',
            self.vehicle_threat_remapped_callback, 10)
        
        self.decision_sub = self.create_subscription(
            String, '/traffic_safety/crossing_decision',
            self.decision_callback, 10)
        
        self.reasoning_sub = self.create_subscription(
            String, '/traffic_safety/decision_reasoning',
            self.reasoning_callback, 10)
        
        # Track message counts
        self.message_counts = {
            'traffic_light_state': 0,
            'traffic_light_state_remapped': 0,
            'traffic_light_confidence': 0,
            'traffic_light_confidence_remapped': 0,
            'vehicle_threat': 0,
            'vehicle_threat_remapped': 0,
            'decision': 0,
            'reasoning': 0
        }
        
        # Track last messages
        self.last_messages = {}
        
        # Timer for periodic status report
        self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('🔍 Decision Engine Debugger started')
        self.get_logger().info('Monitoring all traffic light and decision topics')
    
    def traffic_light_state_callback(self, msg):
        self.message_counts['traffic_light_state'] += 1
        self.last_messages['traffic_light_state'] = msg.data
        self.get_logger().info(f'🚦 Traffic Light State (original): {msg.data}')
    
    def traffic_light_state_remapped_callback(self, msg):
        self.message_counts['traffic_light_state_remapped'] += 1
        self.last_messages['traffic_light_state_remapped'] = msg.data
        self.get_logger().info(f'🚦 Traffic Light State (remapped): {msg.data}')
    
    def traffic_light_confidence_callback(self, msg):
        self.message_counts['traffic_light_confidence'] += 1
        self.last_messages['traffic_light_confidence'] = msg.data
        self.get_logger().info(f'📊 Traffic Light Confidence (original): {msg.data}')
    
    def traffic_light_confidence_remapped_callback(self, msg):
        self.message_counts['traffic_light_confidence_remapped'] += 1
        self.last_messages['traffic_light_confidence_remapped'] = msg.data
        self.get_logger().info(f'📊 Traffic Light Confidence (remapped): {msg.data}')
    
    def vehicle_threat_callback(self, msg):
        self.message_counts['vehicle_threat'] += 1
        self.last_messages['vehicle_threat'] = msg.data
        self.get_logger().info(f'🚨 Vehicle Threat (original): {msg.data}')
    
    def vehicle_threat_remapped_callback(self, msg):
        self.message_counts['vehicle_threat_remapped'] += 1
        self.last_messages['vehicle_threat_remapped'] = msg.data
        self.get_logger().info(f'🚨 Vehicle Threat (remapped): {msg.data}')
    
    def decision_callback(self, msg):
        self.message_counts['decision'] += 1
        self.last_messages['decision'] = msg.data
        self.get_logger().info(f'🧠 Decision: {msg.data}')
    
    def reasoning_callback(self, msg):
        self.message_counts['reasoning'] += 1
        self.last_messages['reasoning'] = msg.data
        self.get_logger().info(f'💭 Reasoning: {msg.data}')
    
    def print_status(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 DECISION ENGINE DEBUG STATUS')
        self.get_logger().info('=' * 60)
        
        for topic, count in self.message_counts.items():
            last_msg = self.last_messages.get(topic, 'None')
            self.get_logger().info(f'{topic}: {count} messages | Last: {last_msg}')
        
        self.get_logger().info('=' * 60)

def main():
    rclpy.init()
    node = DecisionEngineDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Debugger stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 