import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jo-jo/ros2_ws_minimal_pi5/install/robot_legion_teleop_python'
