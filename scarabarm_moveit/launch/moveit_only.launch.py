from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get MoveIt package path
    pkg_path = get_package_share_directory('scarabarm_moveit')

    # Process xacro into URDF
    urdf_file = os.path.join(pkg_path, 'config', 'full_scrab_arm.urdf.xacro')
    initial_positions = os.path.join(pkg_path, 'config', 'initial_positions.yaml')
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_file,
            mappings={'initial_positions_file': initial_positions}
        ).toxml()
    }

    # Load SRDF
    srdf_file = os.path.join(pkg_path, 'config', 'full_scrab_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    # Load MoveIt controller config
    moveit_controllers_yaml = os.path.join(pkg_path, 'config', 'moveit_controllers.yaml')
    planning_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_controllers_yaml
    }

    # Launch move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            planning_config
        ]
    )

    # Launch RViz
    rviz_config_file = os.path.join(pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[robot_description, robot_description_semantic]
    )

    return LaunchDescription([
        move_group_node,
        rviz_node
    ])
