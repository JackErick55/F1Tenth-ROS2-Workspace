import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/darc-f1-01/ros2_ws/install/safety_pkg'
