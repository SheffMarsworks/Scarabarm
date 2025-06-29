from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Official CAN bridge from ros_odrive (adjust bus / IDs as needed)
        Node(
            package='odrive_can',
            executable='odrive_can',
            name='odrive_can',
            parameters=[
                {'can_device': 'can0'},      # example param
            ],
            output='screen'),

        # Your tiny helper
        Node(
            package='odrive_traj_exec',
            executable='arm_axes',
            name='arm_axes',
            output='screen'),
    ])
