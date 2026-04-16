import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/elephant/agvpro_stereo/install/stereo_camera_pkg_py'
