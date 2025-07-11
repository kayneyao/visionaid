import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        
        # Subscribe to RealSense camera topics
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        # Publish to SLAM expected topics
        self.image_pub = self.create_publisher(Image, '/visual_slam/image_0', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/visual_slam/camera_info_0', 10)
        
        self.get_logger().info('Camera bridge node started')
    
    def image_callback(self, msg):
        # Forward image data to SLAM
        self.image_pub.publish(msg)
    
    def info_callback(self, msg):
        # Forward camera info to SLAM
        self.info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = CameraBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
