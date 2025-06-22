from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Point to MoveIt config package
    moveit_pkg = get_package_share_directory('scarabarm_moveit')

    # URDF (xacro -> XML)
    urdf_path = os.path.join(moveit_pkg, 'config', 'full_scrab_arm.urdf.xacro')
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path,
            mappings={'initial_positions_file': initial_positions_file}
        ).toxml()
    }

    # Controller config
    controllers_yaml = os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml')

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    # Spawn controller nodes
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scarab_arm_controller'],
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_spawner,
        arm_controller_spawner
    ])
