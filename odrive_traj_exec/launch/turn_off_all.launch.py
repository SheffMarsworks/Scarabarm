# launch/turn_off_all.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # (Re)start your CAN drivers (if not already running)
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace='odrive_axis0', name='driver0',
            parameters=[{'can_device':'can0'}],
            output='screen'
        ),
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace='odrive_axis1', name='driver1',
            parameters=[{'can_device':'can0'}],
            output='screen'
        ),

        # Call service to set both axes to IDLE (1)
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/odrive_axis0/request_axis_state',
                'odrive_can/srv/AxisState',
                "{axis_requested_state: 1}"
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/odrive_axis1/request_axis_state',
                'odrive_can/srv/AxisState',
                "{axis_requested_state: 1}"
            ],
            output='screen'
        ),
    ])
