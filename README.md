# Autolife_Sim
## Overview

This project aims to build a realistic and scalable robotic simulation environment for the Autolife robot using NVIDIA Isaac Sim and ROS 2. Its primary goal is to provide a high-fidelity platform for developing, testing, and validating mobile manipulation capabilities in complex, human-centered scenarios before deployment in real-world environments.

The workspace includes the core components required for Isaac Sim based ROS 2 integration, including robot description, controller nodes, simulation-side integration, hardware interface support, and sensor related modules. It supports motion control for the mobile base, torso, head, arms, and grippers, while providing a foundation for perception, interaction, and system-level evaluation in simulation.

By combining simulation assets, ROS 2 interfaces, sensors, and modular control components in a single workspace, this project enables rapid prototyping and repeatable evaluation of robotic behaviors in a realistic virtual environment.

## Workspace Structure

The workspace is organized into several ROS 2 packages, each responsible for a different part of the simulation and control stack:

- `autolife_description`: robot description package containing the URDF/Xacro model, meshes, SRDF, RViz configuration, and related assets for visualization and kinematic representation.
- `autolife_control`: ROS 2 controller package providing control nodes for the mobile base, torso, head, arms, and grippers.
- `autolife_sensors`: ROS 2 sensor integration package for the Autolife robot, responsible for configuring and publishing simulated sensor data such as IMU, camera, and LiDAR streams from Isaac Sim to ROS 2 topics.
- `autolife_simulation`: simulation support package for Isaac Sim integration, including configuration files, utility scripts, and response testing tools.
- `autolife_hardware`: `ros2_control` hardware interface package for connecting the robot control stack with simulated or real hardware backends.

Together, these packages form a modular workspace for robot modeling, ROS 2 control, simulation integration, and system evaluation.