import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/het/ros2_ws/src/k1_gemini_follower/install/k1_gemini_follower'
