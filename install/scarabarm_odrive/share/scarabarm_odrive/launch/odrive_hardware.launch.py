#!/usr/bin/env python3
"""
Launch ODrive-based Scarab arm hardware with ros2_control.
"""

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # ──────────────────────────────────────────────────────────────
    # 1. Locate package assets
    # ──────────────────────────────────────────────────────────────
    pkg_path = get_package_share_directory("scarabarm_moveit")

    # --- URDF (processed by xacro so the initial-pose YAML is injected)
    urdf_xacro = os.path.join(pkg_path, "config", "full_scrab_arm.urdf.xacro")
    init_pos   = os.path.join(pkg_path, "config", "initial_positions.yaml")
    robot_description = {
        "robot_description": xacro.process_file(
            urdf_xacro,
            mappings={"initial_positions_file": init_pos}
        ).toxml()
    }

    # --- ros2_control parameter files
    odrive_yaml      = os.path.join(pkg_path, "config", "odrive_system.yaml")
    controllers_yaml = os.path.join(pkg_path, "config", "ros2_controllers.yaml")

    # NB: we *also* want to feed the same controller YAML to the spawner CLI
    #     (that argument is what the joint_trajectory controller reads).
    ctrl_yaml = controllers_yaml   # <── just an alias so the code below is clear

    # ──────────────────────────────────────────────────────────────
    # 2. Nodes
    # ──────────────────────────────────────────────────────────────
    # Publishes TF + /joint_states from the URDF
    robot_state_pub = Node(
        package   = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [robot_description],
        output     = "screen",
    )

    # Main ros2_control node: loads the hardware + all controllers listed
    ros2_control = Node(
        package    = "controller_manager",
        executable = "ros2_control_node",
        parameters = [
            robot_description,      # ← never overrides anything, safe to pass
            odrive_yaml,            # ← ODriveSystem + CAN IDs etc.
            controllers_yaml        # ← controller table (KEEP LAST!)
        ],
        output     = "screen",
    )

    # Spawner helpers  ────────────────────────────────────────────
    spawn_jsb = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = [
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--param-file", ctrl_yaml,
            "--controller-manager-timeout", "50"
        ],
        output     = "screen",
    )

    spawn_arm = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = [
            "scarab_arm_controller",
            "-c", "/controller_manager",
            "--param-file", ctrl_yaml,
            "--controller-manager-timeout", "50"
        ],
        output     = "screen",
    )

    spawn_gripper = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = [
            "gripper_controller",
            "-c", "/controller_manager",
            "--param-file", ctrl_yaml,
            "--controller-manager-timeout", "50"
        ],
        output     = "screen",
    )

    # ──────────────────────────────────────────────────────────────
    # 3. Assemble
    # ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        robot_state_pub,
        ros2_control,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
    ])
