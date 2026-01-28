cmake_minimum_required(VERSION 3.8)
project(my_ur5_control)

cmake_policy(SET CMP0009 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  rclcpp
  moveit_msgs
  moveit_core
  moveit_ros_planning
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# ================================
# Force controller (ros2_control plugin)
# ================================
add_library(force_controller SHARED
  src/force_controller.cpp
)

target_include_directories(force_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(force_controller
  rclcpp
  controller_interface
  hardware_interface
  trajectory_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(
  controller_interface
  force_controller_plugin.xml
)

# ================================
# Planner node
# ================================
add_executable(custom_planner_node
  src/custom_planner_node.cpp
)

ament_target_dependencies(custom_planner_node
  ${dependencies}
)

target_include_directories(custom_planner_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ================================
# Install
# ================================

# Plugin library → lib/
install(
  TARGETS force_controller
  EXPORT export_my_ur5_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executable → lib/<pkg>
install(
  TARGETS custom_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ================================
# Export for downstream packages
# ================================
ament_export_targets(export_my_ur5_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
)

ament_package()
