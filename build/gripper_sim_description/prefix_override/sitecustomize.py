import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/scarab_ros2/src/scarabarm_ros2/install/gripper_sim_description'
