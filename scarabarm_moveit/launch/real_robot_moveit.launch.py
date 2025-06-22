from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


# Helper to load YAML files manually
def load_yaml(package_name, file_path):
    abs_path = os.path.join(
        get_package_share_directory(package_name),
        file_path
    )
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


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
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('scarabarm_moveit', 'config/kinematics.yaml')
    }
    controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')

    # RViz config
    rviz_config_path = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scarab_arm_controller'],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
    )

    # MoveGroup launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'robot_description': robot_description['robot_description'],
            'robot_description_semantic': robot_description_semantic['robot_description_semantic'],
            'robot_description_kinematics': robot_description_kinematics['robot_description_kinematics'],
            'moveit_controller_yaml': moveit_controllers_yaml,
            'rviz_config': rviz_config_path,
            'use_fake_hardware': 'false',
            'use_sim_time': 'false'
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[arm_controller]),
        TimerAction(period=5.0, actions=[gripper_controller]),
        move_group_launch,
        TimerAction(period=6.0, actions=[rviz_node])
    ])
