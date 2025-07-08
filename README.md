# Project Marsworks: Scarab Mars Rover Arm
MATLAB, ROS2 Control, and hardware firmware settings for [Anatolian Rover Challenge (ARC) 2025](https://www.anatolianrover.space/arc-25-missions).

**Main components:**

- Arm Simulation (Gazebo) [under development]
- Arm Teleoperation [under development]
- Arm control (MATLAB + ros2control)

<center> <img src="assets/rover_gazebo_depth.gif" alt="Scarab Rover Simulation in Gazebo"> </center>

## Prerequisites

- ROS2 (Humble or later recommended)
- Ubuntu 22.04 Jammy Jellyfish
- Gazebo Ignition
- MATLAB 2023b
- MATLAB 2023b Toolbox (robotic system toolbox, ROS toolbox, control system toolbox)
- ROS2 control
- ROS2 Package for ODrive

## Installation

1. Install dependencies:
   
  Please install MATLAB 2023b and ROS2

3. Make a folder and clone the repository:
   ```bash
   mkdir Marsworks
   cd Marsworks
   mkdir src
   cd src
   git clone https://github.com/SheffMarsworks/Scarabarm.git
   git clone https://github.com/odriverobotics/ros_odrive.git
   cd ..
   ```

   Make sure to checkout to matlab branch!!!!

   And open Scarabarm/scarab_matlab/ folder and add all subfolder in the path in MATLAB

3. Build the workspace:
   ```bash
   cd Marsworks
   colcon build
   source install/setup.bash
   ```
