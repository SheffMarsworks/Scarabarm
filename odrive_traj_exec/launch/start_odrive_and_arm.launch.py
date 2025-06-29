from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Bring up each CAN node
    for i in range(5):
        ld.add_action(Node(
            package='odrive_can',
            executable='odrive_can_node',
            namespace=f'odrive{i}',
            name='driver',
            parameters=[{'node_id': i, 'can_device': 'can0'}],
            output='screen',
        ))

    # 2) Wait 1s, then start homing (state 6) on all axes
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    f'/odrive{i}/request_axis_state',
                    'odrive_can/srv/AxisState',
                    "{axis_requested_state: 6}"
                ],
                output='screen'
            )
            for i in range(5)
        ]
    ))

    # 3) Wait an additional 5s for homing to finish, then closed-loop (state 8)
    ld.add_action(TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    f'/odrive{i}/request_axis_state',
                    'odrive_can/srv/AxisState',
                    "{axis_requested_state: 8}"
                ],
                output='screen'
            )
            for i in range(5)
        ]
    ))

    # 4) Finally, launch your trajectory follower
    ld.add_action(Node(
        package='odrive_traj_exec',
        executable='traj_follow',
        output='screen',
    ))

    return ld
