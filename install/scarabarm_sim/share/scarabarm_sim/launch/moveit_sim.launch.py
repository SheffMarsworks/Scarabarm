# Launches MoveIt for simulation, sets use_sim_time: true

# Launches MoveIt for simulation:
# • Loads MoveIt configuration (URDF, SRDF, kinematics, controllers, etc.)
# • Starts move_group, RViz, and applies use_sim_time: true

from launch import LaunchDescription
from launch_ros.actions import Node
import os, xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('scarabarm_moveit')
    
    # Process URDF from full_scrab_arm.urdf.xacro with initial positions
    urdf_file = os.path.join(pkg_path, 'config', 'full_scrab_arm.urdf.xacro')
    init_yaml  = os.path.join(pkg_path, 'config', 'initial_positions.yaml')
    robot_description = {
        'robot_description': xacro.process_file(urdf_file, mappings={'initial_positions_file': init_yaml}).toxml()
    }
    
    # Load additional MoveIt files: SRDF, kinematics, controllers YAML (if any)
    srdf_file = os.path.join(pkg_path, 'config', 'full_scrab_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    kinematics_config = {
        'robot_description_kinematics': os.path.join(pkg_path, 'config', 'kinematics.yaml')
    }
    
    planning_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': os.path.join(pkg_path, 'config', 'moveit_controllers.yaml')
    }
    
    sim_time = {'use_sim_time': True}
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[robot_description,
                    robot_description_semantic,
                    kinematics_config,
                    planning_config,
                    sim_time]
    )
    
    rviz_config_file = os.path.join(pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[robot_description, robot_description_semantic, sim_time]
    )
    
    return LaunchDescription([
        move_group_node,
        rviz_node
    ])