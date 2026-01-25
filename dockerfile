# Multi-stage-like from osrf/ros:humble-desktop base 
FROM osrf/ros:humble-desktop

# Core ROS2 setup (your layers 15-20)
SHELL ["/bin/bash", "-c"]

# UR-specific packages + Gazebo/MoveIt (your key layer)
RUN apt update && \
    apt install -y \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-moveit \
      ros-humble-ur \
      ros-humble-ur-robot-driver \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Workspace setup (your final layers)
WORKDIR /ros2_ws
RUN mkdir -p src

# Source ROS on login (your top layer)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Standard ROS entrypoint (inherited)
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
