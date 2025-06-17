# filepath: src/scarabarm_ros2/scarabarm_ros2/launch/gazebo.launch.py
import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_scarabarm_description = get_package_share_directory('scarabarm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the xacro file
    xacro_file_path = os.path.join(
        pkg_scarabarm_description,
        'urdf',
        'scarabarm_urdf.xacro'
    )
    
    # Process the xacro file to get the URDF XML string
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = robot_description_config.toxml()


    # 1. Robot State Publisher Node
    # This node now correctly receives the processed URDF string.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description, # <-- THE FIX IS HERE
        }]
    )

    # 2. Gazebo Simulator Launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r'}.items()
    )

    # 3. Spawn Entity Node (to be launched by the event handler)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scarabarm',
            '-z', '0.1',
            '-allow_renaming', 'true'
        ]
    )

    # 4. Event Handler to delay the spawner
    # This ensures the robot is spawned only after the robot_state_publisher is ready.
    spawn_delay_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[spawn_entity_node],
        )
    )

    # Launch Description
    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher_node,
        spawn_delay_handler,
    ])