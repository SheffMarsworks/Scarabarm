from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('scarabarm_moveit')

    robot_description = {
        'robot_description': Command([
            'xacro ',
            os.path.join(pkg_path, 'urdf', 'full_scrab_arm.urdf.xacro')
        ])
    }

    controllers_yaml = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scarab_arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])