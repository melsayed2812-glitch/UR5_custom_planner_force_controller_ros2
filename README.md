# UR5 ROS2 Force-Control Project

**Master Project 2**  
UR5 simulation with a custom Cartesian path planner and PD-based force controller using **ROS 2 Humble** and **Gazebo**.

This repository is structured for **academic review and reproducibility**, with validation scenarios corresponding to **Chapters 6–7** of the report.

---

## System Overview
- **Robot**: UR5 (UR5-compatible kinematics for simulation)
- **Planner**: Cartesian end-effector trajectory generator
- **Controller**:Controller: Joint-space PD controller with external force disturbance injection
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
git clone https://github.com/melsayed2812-glitch/UR5_custom_planner_force_controller_ros2.git
cd UR5_custom_planner_force_controller_ros2

docker build -t ur_force_project .
docker run -it --rm --privileged -p 6080:6080 \
  -e DISPLAY=$DISPLAY ur_force_project
```
> Docker ensures deterministic results during academic evaluation.

---

## Launch Simulation
Open a **new terminal inside the container**:

colcon build (if needed)
source install/setup.bash
apt update && apt install -y \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  ros-humble-ros2doctor



ros2 launch my_ur5_control fake_my_ur5.launch.py ur_type:=ur5
Supported robot models:
- `ur5` (default)
- `ur6` (if enabled in URDF/xacro)

---

## Essential Monitoring (End-Effector Verification)
These commands are **recommended for academic review** to verify tracking accuracy.

# Joint states (low-level verification)
ros2 topic echo /joint_states

# End-effector reference (goal)
ros2 topic echo /ee_goal

# # End-effector actual pose (forward kinematics)
ros2 topic echo /ee_pose

# Planner trajectory output
ros2 topic echo /my_force_controller/trajectory

# Applied external force
ros2 topic echo /fake_force
```
Visualization:

rviz2
```
Recommended RViz displays:
- RobotModel
- TF
- Pose (EE goal vs EE actual)
- Path (planner output)

---

## Validation Scenarios (Reproduce Report Ch. 6 & 7)

### Scenario 1: Free-Space Motion (No External Force)
```bash
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {position: {x: 0.2, y: 0.0, z: 0.5}, orientation: {w: 1.0}}
}" --once
```
Expected:
- RMS Cartesian error < **2 mm**
- Smooth velocity profile
Repeat for:
- `x = 0.4` (Configuration B)
- `x = 0.7` (Configuration C)

---

### Scenario 2: Transient Disturbance Rejection
**Start motion**
```bash
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0}}
}" --once
```
**Apply disturbance (~2 s mid-trajectory)**
```bash
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [7.0, 0.0, 0.0]}" --once
```
**Remove disturbance**
```bash
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0]}" --once
```
Expected:
- Peak deviation: **5–15 mm**
- Recovery time: **< 400 ms**

---

### Scenario 3: Persistent External Force
```bash
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [5.0, 0.0, 0.0]}" --once
```
```bash
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{
  pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0}}
}" --once
```
Expected:
- Steady-state offset: **6–8 mm**
- Stable trajectory completion

---

## Recommended Commands for Academic Review
```bash
ros2 doctor --report
ros2 node list
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /my_force_controller/trajectory
```
Used to verify:
- Correct node initialization
- Deterministic controller execution
- Real-time topic rates

---

## Notes for Examiners
- All experiments are reproducible using Docker
- No hardware dependencies


---

## Author
**Mohamed Nassar**  
M.Sc. Mechatronics Engineering

