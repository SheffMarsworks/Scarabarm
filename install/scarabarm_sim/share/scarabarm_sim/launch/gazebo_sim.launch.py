#  Spawns your robot URDF in Gazebo, loads ros2_control simulation config, spawns controllers.

# Launches Gazebo:
# • Loads the robot description (URDF generated from a Xacro file)
# • Starts robot_state_publisher, the ros2_control_node using the simulation config,
# • Spawns the robot into the Gazebo world and bridges topics (if needed)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sim        = get_package_share_directory('scarabarm_sim')
    
    # Process the full arm Xacro file
    robot_description_file = os.path.join(pkg_sim, 'urdf', 'full_scarab_arm.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # (Optional) Provide a bridge config if needed
    ros_gz_bridge_config = os.path.join(pkg_sim, 'config', 'ros_gz_bridge_sim.yaml')
    
    # Start the robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # Start the ros2_control_node using simulation config (controllers_sim.yaml)
    sim_ros2_control_config = os.path.join(pkg_sim, 'config', 'controllers_sim.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, sim_ros2_control_config],
        output='screen'
    )
    
    # Include Gazebo simulation (for example via gz_sim.launch.py)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )
    
    # Spawn the robot into Gazebo using the /robot_description topic
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "FullScarabArm",
            "-allow_renaming", "true",
            "-z", "0.32",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ],
        output='screen'
    )
    
    # (Optional) Bridge ROS and Gazebo topics; update bridge config if you have one
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn,
        ros2_control_node,
        robot_state_publisher,
        bridge
    ])