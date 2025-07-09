from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg_share = get_package_share_directory("scarabarm_control")

    # ★ build URDF ----------------------------------------------------------------
    moveit_pkg = get_package_share_directory("scarabarm_moveit")
    urdf_xacro = os.path.join(moveit_pkg, "config", "full_scrab_arm.urdf.xacro")
    init_yaml  = os.path.join(moveit_pkg, "config", "initial_positions.yaml")
    robot_xml  = xacro.process_file(
        urdf_xacro, mappings={"initial_positions_file": init_yaml}).toxml()
    # ★ ---------------------------------------------------------------------------

    hw_yaml   = os.path.join(pkg_share, "config", "ros2_control_odrive.yaml")

    scarab_hw = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_xml}, hw_yaml],   # ★ pass URDF, then YAML
        output="screen"
    )

    js_broadcaster = Node(   # ★ must come first
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    traj_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scarab_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    js_agg = Node(
        package="scarabarm_control",
        executable="scarab_js_agg",
        name="joint_state_aggregator",
        output="screen"
    )

    return LaunchDescription([
        Node(package="robot_state_publisher",      # ★ publishes /robot_description
             executable="robot_state_publisher",
             parameters=[{"robot_description": robot_xml}]),
        scarab_hw,
        js_broadcaster,
        traj_ctrl,
        js_agg,
    ])
