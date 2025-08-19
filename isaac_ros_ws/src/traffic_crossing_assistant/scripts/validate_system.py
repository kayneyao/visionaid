#!/usr/bin/env python3
"""Quick system validation script - Updated for 11-class, 2-priority system"""

import rclpy
from rclpy.node import Node
import subprocess
import time

class SystemValidator(Node):
    def __init__(self):
        super().__init__('system_validator')
        
    def validate_nodes(self):
        """Check if all required nodes are available"""
        required_executables = [
            'vehicle_movement_analyzer',
            'crosswalk_analyzer', 
            'traffic_light_analyzer',
            'decision_engine',
            'audio_feedback_system',
            'enhanced_audio_system',
            'multimodal_safety_coordinator'
        ]
        
        self.get_logger().info('Validating 11-class traffic safety system...')
        
        for executable in required_executables:
            try:
                result = subprocess.run(
                    ['ros2', 'pkg', 'executables', 'traffic_crossing_assistant'],
                    capture_output=True, text=True, timeout=5
                )
                if executable in result.stdout:
                    self.get_logger().info(f'{executable} - Available')
                else:
                    self.get_logger().error(f'{executable} - Missing')
            except Exception as e:
                self.get_logger().error(f'Validation failed: {e}')

def main():
    rclpy.init()
    validator = SystemValidator()
    validator.validate_nodes()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
