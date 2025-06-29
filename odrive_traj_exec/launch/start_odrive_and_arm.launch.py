# launch/close_loop_all.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the CAN bridge (example namespace for two axes)
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace='odrive_axis0', name='driver0',
            parameters=[{'can_device':'can0'}]
        ),
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace='odrive_axis1', name='driver1',
            parameters=[{'can_device':'can0'}]
        ),

        # Then immediately request CLOSED_LOOP_CONTROL on both axes:
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/odrive_axis0/request_axis_state',
                'odrive_can/srv/AxisState',
                "{axis_requested_state: 8}"
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/odrive_axis1/request_axis_state',
                'odrive_can/srv/AxisState',
                "{axis_requested_state: 8}"
            ],
            output='screen'
        ),
    ])
