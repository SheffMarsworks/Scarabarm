from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():
    # Locate MoveIt config package
    moveit_config_pkg = get_package_share_directory('scarabarm_moveit')

    # Process xacro to generate URDF
    xacro_path = os.path.join(moveit_config_pkg, 'config', 'full_scrab_arm.urdf.xacro')
    xacro_config = xacro.process_file(
        xacro_path,
        mappings={'initial_positions_file': os.path.join(moveit_config_pkg, 'config', 'initial_positions.yaml')}
    )
    robot_description = {'robot_description': xacro_config.toxml()}

    # SRDF
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'full_scrab_arm.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics and controllers
    kinematics_yaml = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')

    # RViz config
    rviz_config_path = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')

    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )


    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Arm controller
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scarab_arm_controller'],
    )

    # Gripper controller
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
    )

    # MoveGroup
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'robot_description': robot_description['robot_description'],
            'robot_description_semantic': robot_description_semantic['robot_description_semantic'],
            'kinematics_yaml': kinematics_yaml,
            'moveit_controller_yaml': moveit_controllers_yaml,
            'rviz_config': rviz_config_path,
            'use_fake_hardware': 'false',
            'use_sim_time': 'false'
        }.items()
    )


    # Explicitly launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,  # <-- ADD THIS LINE
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[arm_controller]),
        TimerAction(period=5.0, actions=[gripper_controller]),
        move_group_launch,
        TimerAction(period=6.0, actions=[rviz_node])
    ])

