# launch_odrives.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    for axis_id in range(5):  # 0 through 4 for joints 2-6
        ld.add_action(Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace=f'odrive_axis{axis_id}',
            name='odrive_can_node',
            parameters=[{
                'node_id': axis_id,       # ODrive CAN node ID
                'interface': 'can0'       # CAN bus interface name
            }]
        ))
    return ld
