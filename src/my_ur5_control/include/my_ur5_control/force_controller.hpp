#pragma once

#include <vector>
#include <string>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace my_ur5_control
{

class ForceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Trajectory handling
  void trajCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  bool sampleTrajectory(double t, std::vector<double> & q_des, std::vector<double> & qd_des);
  void clearTrajectory();

  // Fake force input
  void fakeForceCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<double> kp_;
  std::vector<double> kd_;

  // Trajectory data
  trajectory_msgs::msg::JointTrajectory current_traj_;
  rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};
  bool has_traj_{false};

  // Subscriptions
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fake_force_sub_;

  // Fake wrench Fx,Fy,Fz,Tx,Ty,Tz
  std::vector<double> fake_force_;  // size 6

  // Simple gain to map Fx into extra torque on joint 0
  double fake_force_gain_{1.0};
};

}  