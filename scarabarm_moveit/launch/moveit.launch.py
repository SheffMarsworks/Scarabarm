from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg_moveit = get_package_share_directory('scarabarm_moveit')

    # URDF Xacro · SRDF · 기본 자세 yaml
    urdf = os.path.join(pkg_moveit, 'config', 'full_scrab_arm.urdf.xacro')
    init = os.path.join(pkg_moveit, 'config', 'initial_positions.yaml')
    srdf = os.path.join(pkg_moveit, 'config', 'full_scrab_arm.srdf')

    robot_description = {
        'robot_description': xacro.process_file(
            urdf, mappings={'initial_positions_file': init}).toxml()
    }
    robot_description_semantic = {
        'robot_description_semantic': open(srdf).read()
    }

    # MoveIt 설정 yaml 경로
    kin_yaml   = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_yaml  = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    traj_yaml  = os.path.join(pkg_moveit, 'config', 'trajectory_execution.yaml')
    psm_yaml   = os.path.join(pkg_moveit, 'config', 'planning_scene_monitor.yaml')
    ctrl_yaml  = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')

    node_move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kin_yaml},
            ompl_yaml,
            traj_yaml,
            psm_yaml,
            {'moveit_controller_manager':
             'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            ctrl_yaml
        ])

    node_rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[robot_description, robot_description_semantic])

    return LaunchDescription([node_move_group, node_rviz])
