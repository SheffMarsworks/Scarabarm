from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name='can_node_1',
            namespace='odrive_axis0',
            parameters=[{'node_id': 1, 'interface': 'can0'}]
        ),
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name='can_node_2',
            namespace='odrive_axis1',
            parameters=[{'node_id': 2, 'interface': 'can0'}]
        )
    ])
