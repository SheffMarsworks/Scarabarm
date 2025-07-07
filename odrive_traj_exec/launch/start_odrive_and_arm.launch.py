#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Bring up one CAN bridge per ODrive
    for i in range(5):
        ld.add_action(Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace=f'odrive{i}',
            name='driver',
            parameters=[{'node_id': i, 'can_device': 'can0'}],
            output='screen',
        ))

    # 2) After 1s: Encoder Offset Calibration
    for i in range(5):
        ld.add_action(TimerAction(
            period=1.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    f'/odrive{i}/request_axis_state',
                    'odrive_can/srv/AxisState',
                    '{axis_requested_state: 7}'
                ],
                output='screen'
            )]
        ))

    # 3) After 10s: close loop mode
    for i in range(5):
        ld.add_action(TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    f'/odrive{i}/request_axis_state',
                    'odrive_can/srv/AxisState',
                    '{axis_requested_state: 8}'
                ],
                output='screen'
            )]
        ))

    # 4) After 10s: Command position 0.0
    for i in range(5):
        ld.add_action(TimerAction(
            period=15.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-1',
                    f'/odrive{i}/control_message',
                    'odrive_can/msg/ControlMessage',
                    '{control_mode: 3, input_mode: 5, input_pos: 0.0, input_vel: 0.0, input_torque: 0.0}'
                ],
                output='screen'
            )]
        ))

    # 5) After 18s: Start trajectory follower (if you want)
    ld.add_action(TimerAction(
        period=25.0,
        actions=[Node(
            package='odrive_traj_exec',
            executable='traj_follow',
            output='screen',
        )]
    ))

    # 6) After 25s: Start joint position executor node
    ld.add_action(TimerAction(
        period=26.0,
        actions=[Node(
            package='odrive_traj_exec',
            executable='position_executor',
            output='screen',
        )]
    ))

    return ld
