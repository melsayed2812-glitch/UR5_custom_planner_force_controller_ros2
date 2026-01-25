# UR6 ROS2 Force-Control Project

Master project 2: UR6 simulation with custom path planner + force controller in ROS2 Humble + Gazebo.[file:1]

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 (via `ros-humble-gazebo-ros-pkgs`)
- Docker (recommended for reproducibility)

## Quick Start (Docker)
```bash
git clone [https://github.com/melsayed2812-glitch/UR6-ROS2-Force-Control.git](https://github.com/melsayed2812-glitch/UR6-ROS2-Force-Control.git)
cd UR6-ROS2-Force-Control/ros2_ws
docker build -t ur_force_project .
docker run -it --rm --privileged -p 6080:6080 -e DISPLAY=$DISPLAY ur_force_project  # X11 forwarding


## Validation Scenarios (Reproduce Report Ch.6/7)

**Launch sim** (new terminal each time):  
```bash
source install/setup.bash
ros2 launch my_ur6_control fake_my_ur6.launch.py ur_type:=ur5  # or ur6[1]
ros2 topic echo /joint_states          # Track positions/errors
ros2 topic echo /ee_goal               # Goals
ros2 topic echo /fake_force            # Disturbances
ros2 topic echo /my_force_controller/trajectory  # Planner output
ros2 run rviz2 rviz2                   # Viz robot/trajectory
##Scenario 1: Basic Move (No Force)
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {position: {x: 0.2, y: 0.0, z: 0.5}, orientation: {w: 1.0}}
}" --once  # Config A, expect <2mm RMS[1]

# Repeat: x: 0.4 (Config B), x: 0.7 (Config C)
##Scenario 2: Transient Disturbance

# Start move (x=0.2)
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{position: {x: 0.2, y: 0.0, z: 0.5}, orientation: {w: 1.0}}" --once

# ~2s later (mid-trajectory): Apply 10N force
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [10.0, 0.0, 0.0]}" --once  # Fx=10N[1]

# ~2s later: Remove
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0]}" --once

#Expect: 5-15mm deviation → 400ms recovery.

##Scenario 3: Persistent Force

# Continuous 5N resistance FIRST
ros2 topic pub /fake_force std_msgs/Float64MultiArray "{data: [5.0, 0.0, 0.0]}" --once

# Then goal
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "{position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {w: 1.0}}" --once
#Expect: Steady 6-8mm offset, completes trajectory