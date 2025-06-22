from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bring up ros2_control_node with robot description and controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': open('<path_to>/full_scrab_arm.urdf.xacro').read()},
                '<path_to>/ros2_controllers.yaml'
            ],
            output='screen'
        ),

        # Spawners for controllers
        Node(package='controller_manager', executable='spawner', arguments=['scarab_arm_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),

        # Launch ODrive CAN nodes
        Node(package='odrive_can', executable='odrive_can_node', name='odrive_axis2', parameters=[{'node_id': 2, 'interface': 'can0'}]),
        Node(package='odrive_can', executable='odrive_can_node', name='odrive_axis3', parameters=[{'node_id': 3, 'interface': 'can0'}]),
        Node(package='odrive_can', executable='odrive_can_node', name='odrive_axis4', parameters=[{'node_id': 4, 'interface': 'can0'}]),
        Node(package='odrive_can', executable='odrive_can_node', name='odrive_axis5', parameters=[{'node_id': 5, 'interface': 'can0'}]),
        Node(package='odrive_can', executable='odrive_can_node', name='odrive_axis6', parameters=[{'node_id': 6, 'interface': 'can0'}]),

        # Launch trajectory executor
        Node(
            package='odrive_node',
            executable='odrive_trajectory_executor.py',
            name='odrive_trajectory_executor',
            output='screen'
        )
    ])
