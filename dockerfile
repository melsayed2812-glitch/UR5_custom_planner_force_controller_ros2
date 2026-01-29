# Base ROS2 Humble desktop image
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# Install required ROS2 packages
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

# Set ROS2 workspace
WORKDIR /ros2_ws

# Copy your local repository into the container's workspace
COPY ./src /ros2_ws/src

# Source ROS2 and build all packages in src
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Automatically source ROS2 and workspace on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Standard ROS entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


