# UR6 ROS2 Force-Control Project

**Master Project 2**  
UR6 simulation with a custom Cartesian path planner and PD-based force controller using **ROS 2 Humble** and **Gazebo**.

This repository is structured for **academic review and reproducibility**, with validation scenarios corresponding to **Chapters 6–7** of the report.

---

## System Overview
- **Robot**: UR6 (UR5-compatible kinematics for simulation)
- **Planner**: Cartesian end-effector trajectory generator
- **Controller**: PD force controller with external disturbance injection
- **Simulation**: Gazebo + fake force interface
- **Visualization**: RViz2

---

## Requirements
- Ubuntu 22.04  
- ROS 2 Humble  
- Gazebo 11 (`ros-humble-gazebo-ros-pkgs`)  
- Docker (recommended for reproducibility)

---

## Quick Start (Docker – Recommended)

```bash
git clone https://github.com/melsayed2812-glitch/UR6-ROS2-Force-Control.git
cd UR6-ROS2-Force-Control/ros2_ws

docker build -t ur_force_project .
docker run -it --rm --privileged -p 6080:6080 \
  -e DISPLAY=$DISPLAY ur_force_project
