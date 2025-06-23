from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('scarabarm_moveit')

    # 경로 단축
    robot_description     = os.path.join(pkg_share, 'config', 'urdf', 'scarabarm.urdf.xacro')
    robot_description_sem = os.path.join(pkg_share, 'config', 'urdf', 'scarabarm.srdf')
    kinematics_yaml       = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    ompl_planning_yaml    = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    controllers_yaml      = os.path.join(pkg_share, 'config', 'moveit_controllers.yaml')
    trajectory_yaml       = os.path.join(pkg_share, 'config', 'trajectory_execution.yaml')
    planning_scene_yaml   = os.path.join(pkg_share, 'config', 'planning_scene_monitor.yaml')

    # move_group 노드
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {"robot_description":             open(robot_description, 'r').read()},
            {"robot_description_semantic":    open(robot_description_sem, 'r').read()},
            {"robot_description_kinematics":  kinematics_yaml},
            ompl_planning_yaml,
            trajectory_yaml,
            planning_scene_yaml,
            {"moveit_controller_manager":
             "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            controllers_yaml                 # ← YAML 통째로 읽어도 되고
        ],
    )

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
        parameters=[
            {"robot_description":          open(robot_description, 'r').read()},
            {"robot_description_semantic": open(robot_description_sem, 'r').read()},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
