import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lephi7/Documents/ros2_projects/laser_test/install/laser_pos'
