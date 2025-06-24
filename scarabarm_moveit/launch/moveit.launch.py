from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml, tempfile
from moveit_configs_utils import MoveItConfigsBuilder   # already in your venv
from launch.substitutions import PathJoinSubstitution        # correct module
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("scarabarm_moveit")

    # --- URDF / SRDF --------------------------------------------------------
    urdf = os.path.join(pkg, "config", "full_scrab_arm.urdf.xacro")
    srdf = os.path.join(pkg, "config", "full_scrab_arm.srdf")
    init = os.path.join(pkg, "config", "initial_positions.yaml")
    robot_description = {
        "robot_description": xacro.process_file(
            urdf, mappings={"initial_positions_file": init}).toxml()
    }
    robot_description_semantic = {
        "robot_description_semantic": open(srdf).read()
    }

    # --- kinematics (keep path) --------------------------------------------
    # kin_yaml  = os.path.join(pkg, "config", "kinematics.yaml")
    # ompl_yaml  = os.path.join(pkg, "config", "ompl_planning.yaml")
    kinematics_yaml = PathJoinSubstitution([FindPackageShare("scarabarm_moveit"),'config','kinematics_ros2.yaml',])
    ompl_yaml = PathJoinSubstitution([FindPackageShare("scarabarm_moveit"), "config", "ompl_planning_ros2.yaml"])

    # --- load your unchanged controller YAML --------------------------------
    ctrl_path = os.path.join(pkg, "config", "moveit_controllers.yaml")
    with open(ctrl_path, "r") as f:
        raw_yaml = yaml.safe_load(f)

    ctrl_list = (raw_yaml["controller_list"]
                 if isinstance(raw_yaml, dict) else raw_yaml)

    # build MoveIt-2 style param tree  (all scalars / dicts → rcl friendly)
    msc_param = {"controller_names": []}
    for c in ctrl_list:
        name = c["name"]
        msc_param["controller_names"].append(name)
        msc_param[name] = {
            "action_ns": c["action_ns"],
            "type":      c["type"],
            "default":   bool(c.get("default", False)),
            "joints":    c["joints"],
        }

    # --- tiny wrapper file that rcl CAN parse -------------------------------
    tmp_file = tempfile.NamedTemporaryFile("w", delete=False, suffix=".yaml")
    yaml.safe_dump({
        "move_group": {
            "ros__parameters": {
                "moveit_controller_manager":
                  "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": msc_param   # ← here
            } } }, tmp_file)
    tmp_file.close()

    # --- TF publishers ------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description])

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen")

    # --- MoveIt & RViz ------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,      # ← now a **file**, not a literal string
            ompl_yaml,
            tmp_file.name
        ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg, "config", "moveit.rviz")],
        output="screen",
        parameters=[robot_description, robot_description_semantic])

    return LaunchDescription([static_tf, rsp, move_group, rviz])
