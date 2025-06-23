#!/usr/bin/env python3
"""
Launch ODrive-based Scarab arm hardware with ros2_control.
"""

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. Locate package assets
    # ------------------------------------------------------------------
    pkg_path = get_package_share_directory("scarabarm_moveit")

    # --- URDF ---------------------------------------------------------
    urdf_xacro = os.path.join(pkg_path, "config", "full_scrab_arm.urdf.xacro")
    init_pos   = os.path.join(pkg_path, "config", "initial_positions.yaml")

    robot_description = {
        "robot_description": xacro.process_file(
            urdf_xacro, mappings={"initial_positions_file": init_pos}
        ).toxml()
    }

    # --- YAML parameter files ----------------------------------------
    odrive_yaml      = os.path.join(pkg_path, "config", "odrive_system.yaml")
    controllers_yaml = os.path.join(pkg_path, "config", "ros2_controllers.yaml")

    # ------------------------------------------------------------------
    # 2. Nodes
    # ------------------------------------------------------------------
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,   # URDF (never overrides anything)
            odrive_yaml,         # hardware & CAN settings
            controllers_yaml,    # controller table (keep LAST)
        ],
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "50"],
        output="screen",
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scarab_arm_controller", "--controller-manager-timeout", "50"],
        output="screen",
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "50"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 3. Assemble launch description
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            robot_state_pub,
            ros2_control,
            spawn_jsb,
            spawn_arm,
            spawn_gripper,
        ]
    )
