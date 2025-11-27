#!/usr/bin/env python3
# serial_imu_min.py — ROS params only

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
import serial

class SerialImuMin(Node):
    def __init__(self):
        super().__init__('serial_imu_min')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('period', 0.01)          # seconds (e.g., 0.01 -> 100 Hz)
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('frame_mag', 'mag_link')
        self.declare_parameter('mag_units', 't')       # 'ut' or 't'
        self.declare_parameter('request_byte', 'j')
        self.declare_parameter('read_timeout', 0.008)   # seconds to wait for a reply

        port         = self.get_parameter('port').value
        baud         = int(self.get_parameter('baud').value)
        period       = float(self.get_parameter('period').value)
        self.frame_imu = self.get_parameter('frame_imu').value
        self.frame_mag = self.get_parameter('frame_mag').value
        self.mag_units = str(self.get_parameter('mag_units').value).lower()
        req_byte_str = str(self.get_parameter('request_byte').value)
        read_timeout = float(self.get_parameter('read_timeout').value)

        self.req_byte = req_byte_str.encode('ascii', errors='ignore') or b'j'

        # ---- Serial ----
        try:
            self.ser = serial.Serial(port, baud, timeout=read_timeout)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Opened {port} @ {baud}; requesting with "{req_byte_str}" every {period*1000:.1f} ms')
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial: {e}')
            raise

        # ---- Publishers (SensorDataQoS) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_imu = self.create_publisher(Imu, 'imu', qos)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', qos)

        # ---- Timer ----
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        try:
            # 1) Ask for one sample
            self.ser.write(self.req_byte)

            # 2) Read one CSV line
            line = self.ser.readline()
            if not line:
                return

            stamp = self.get_clock().now()  # arrival time

            s = line.decode('utf-8', errors='ignore').strip()
            parts = s.split(',')
            if len(parts) != 9:
                return

            ax, ay, az, gx, gy, gz, mx, my, mz = map(float, parts)

            # Magnetometer units → Tesla
            if self.mag_units == 'ut':
                mx *= 1e-6; my *= 1e-6; mz *= 1e-6

            # OPTIONAL: axis remap (edit if your board isn't already ENU)
            # ax, ay, az = ax, ay, az
            # gx, gy, gz = gx, gy, gz
            # mx, my, mz = mx, my, mz

            # 3) Build messages
            imu_msg = Imu()
            imu_msg.header.stamp = stamp.to_msg()
            imu_msg.header.frame_id = self.frame_imu
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = self.frame_mag
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz

            # 4) Publish
            self.pub_imu.publish(imu_msg)
            self.pub_mag.publish(mag_msg)

        except Exception as e:
            self.get_logger().warn(f'serial tick error: {e}')

def main(args=None):
    rclpy.init(args=args)  # handles --ros-args, remaps, etc.
    node = SerialImuMin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
