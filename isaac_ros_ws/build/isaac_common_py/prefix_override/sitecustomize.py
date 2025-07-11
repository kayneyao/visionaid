import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sophie/visionaid/isaac_ros_ws/install/isaac_common_py'
