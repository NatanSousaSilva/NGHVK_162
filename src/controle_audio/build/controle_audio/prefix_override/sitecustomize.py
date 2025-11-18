import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/natan/ros2_ws/src/controle_audio/install/controle_audio'
