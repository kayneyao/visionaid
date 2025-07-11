import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sophie/visionaid/isaac_ros_ws/install/traffic_crossing_assistant'
