import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan-cito/tec/ros_ws_hw1/install/planar_target_tracker'
