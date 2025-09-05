import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luka/tf_luna_ros2_ws/install/tfluna_ros2'
