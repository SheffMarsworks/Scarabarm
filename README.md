# Project Marsworks: Scarab Mars Rover Arm
ROS2 Moveit!, ROS2 Control, and hardware firmware settings for [Anatolian Rover Challenge (ARC) 2025](https://www.anatolianrover.space/arc-25-missions).

**Main components:**

- Arm Simulation (Gazebo) [under development]
- Arm Teleoperation [under development]
- Arm control (MoveIt!2 + ros2control)

<center> <img src="assets/rover_gazebo_depth.gif" alt="Scarab Rover Simulation in Gazebo"> </center>

## Prerequisites

- ROS2 (Humble or later recommended)
- Ubuntu 22.04 Jammy Jellyfish
- Gazebo Ignition
- MoveIt!2 humble
- ROS2 control
- ROS2 Package for ODrive

## Installation

1. Install dependencies:
   
  Please follow this page for MoveIt!2: https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

2. Make a folder and clone the repository:
   ```bash
   mkdir Marsworks
   cd Marsworks
   mkdir src
   cd src
   git clone https://github.com/SheffMarsworks/Scarabarm.git
   git clone https://github.com/odriverobotics/ros_odrive.git
   cd ..
   ```

3. Build the workspace:
   ```bash
   cd Marsworks
   colcon build
   source install/setup.bash
   ```
