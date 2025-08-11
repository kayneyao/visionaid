#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

class ImuDtMonitor(Node):
    def __init__(self):
        super().__init__('imu_dt_monitor')
        # change topic name / queue size as needed
        self.sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        ))
        self.last_time = None
        self.dts = []

    def imu_callback(self, msg: Imu):
        # convert header stamp to floating‐point seconds
        t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) * 1e3
        if self.last_time is not None:
            self.dts.append(t - self.last_time)
        self.last_time = t

    def on_shutdown(self):
        if not self.dts:
            self.get_logger().warn("No IMU messages received, no data to plot.")
            return

        # scatter plot dt vs. index
        plt.figure(figsize=(8,4))
        plt.scatter(range(len(self.dts)), self.dts, s=5)
        plt.xlabel('Sample index')
        plt.ylabel('Δt (ms)')
        plt.title('IMU Δt over time')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ImuDtMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
