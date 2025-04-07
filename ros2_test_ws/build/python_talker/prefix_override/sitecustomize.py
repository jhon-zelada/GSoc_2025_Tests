import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jhon/GSC_2025_test/ros2_test_ws/install/python_talker'
