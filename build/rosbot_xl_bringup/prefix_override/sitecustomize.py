import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sampathkumaritagi/ros_ws/install/rosbot_xl_bringup'
