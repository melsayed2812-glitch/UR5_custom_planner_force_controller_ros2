#include "my_ur5_control/force_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace my_ur5_control
{


using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;


InterfaceConfiguration ForceController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}


InterfaceConfiguration ForceController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}


CallbackReturn ForceController::on_init()
{
  return CallbackReturn::SUCCESS;
}


CallbackReturn ForceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();


  node->get_parameter("joints", joint_names_);
  node->get_parameter("kp", kp_);
  node->get_parameter("kd", kd_);
  node->get_parameter("fake_force_gain", fake_force_gain_);
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "ForceController: 'joints' parameter is empty");
    return CallbackReturn::ERROR;
  }


  if (kp_.empty()) {
    kp_.assign(joint_names_.size(), 50.0);
  }
  if (kd_.empty()) {
    kd_.assign(joint_names_.size(), 2.0);
  }


  if (kp_.size() != joint_names_.size() || kd_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ForceController: kp/kd size must match joints size");
    return CallbackReturn::ERROR;
  }


  fake_force_.assign(6, 0.0);


  RCLCPP_INFO(
    node->get_logger(),
    "ForceController configured with %zu joints", joint_names_.size());


  return CallbackReturn::SUCCESS;
}


CallbackReturn ForceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();


  if (state_interfaces_.size() != 2 * joint_names_.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ForceController: expected %zu state interfaces, got %zu",
      2 * joint_names_.size(), state_interfaces_.size());
    return CallbackReturn::ERROR;
  }


  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ForceController: expected %zu command interfaces, got %zu",
      joint_names_.size(), command_interfaces_.size());
    return CallbackReturn::ERROR;
  }


  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }


  traj_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/trajectory", 10,
    std::bind(&ForceController::trajCb, this, std::placeholders::_1));


  fake_force_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/fake_force", 10,
    std::bind(&ForceController::fakeForceCb, this, std::placeholders::_1));


  has_traj_ = false;


  RCLCPP_INFO(node->get_logger(), "ForceController activated");


  return CallbackReturn::SUCCESS;
}


CallbackReturn ForceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }
  clearTrajectory();
  traj_sub_.reset();
  fake_force_sub_.reset();
  return CallbackReturn::SUCCESS;
}


void ForceController::clearTrajectory()
{
  has_traj_ = false;
  current_traj_.points.clear();
}


void ForceController::trajCb(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  auto node = get_node();


  if (msg->points.empty()) {
    RCLCPP_WARN(node->get_logger(), "ForceController: received empty trajectory");
    return;
  }


  if (!msg->joint_names.empty() &&
      msg->joint_names.size() != joint_names_.size()) {
    RCLCPP_WARN(
      node->get_logger(),
      "ForceController: trajectory joints size %zu does not match controller joints size %zu",
      msg->joint_names.size(), joint_names_.size());
  }


  current_traj_ = *msg;
  traj_start_time_ = node->now();
  has_traj_ = true;


  RCLCPP_INFO(
    node->get_logger(),
    "ForceController: received trajectory with %zu points",
    current_traj_.points.size());
}


void ForceController::fakeForceCb(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() == 6) {
    fake_force_ = msg->data;
  }
}


bool ForceController::sampleTrajectory(
  double t,
  std::vector<double> & q_des,
  std::vector<double> & qd_des)
{
  const auto & pts = current_traj_.points;
  if (pts.empty()) {
    return false;
  }


  if (pts.size() == 1 || t <= 0.0) {
    q_des = pts.front().positions;
    qd_des.assign(q_des.size(), 0.0);
    return true;
  }


  for (size_t i = 1; i < pts.size(); ++i) {
    double t_i = pts[i].time_from_start.sec +
                 pts[i].time_from_start.nanosec * 1e-9;
    if (t <= t_i) {
      const auto & p0 = pts[i - 1];
      const auto & p1 = pts[i];


      double t0 = p0.time_from_start.sec +
                  p0.time_from_start.nanosec * 1e-9;


      double alpha = (t - t0) / (t_i - t0);
      if (alpha < 0.0) alpha = 0.0;
      if (alpha > 1.0) alpha = 1.0;


      q_des.resize(p0.positions.size());
      qd_des.assign(p0.positions.size(), 0.0);


      for (size_t j = 0; j < p0.positions.size(); ++j) {
        double q0 = p0.positions[j];
        double q1 = p1.positions[j];
        q_des[j] = q0 + alpha * (q1 - q0);
      }
      return true;
    }
  }


  return false;
}


return_type ForceController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  if (!has_traj_) {
    for (auto & cmd : command_interfaces_) {
      cmd.set_value(0.0);
    }
    return return_type::OK;
  }


  double t = (time - traj_start_time_).seconds();


  std::vector<double> q_des;
  std::vector<double> qd_des;
  if (!sampleTrajectory(t, q_des, qd_des)) {
    clearTrajectory();
    for (auto & cmd : command_interfaces_) {
      cmd.set_value(0.0);
    }
    return return_type::OK;
  }


  for (size_t i = 0; i < joint_names_.size(); ++i) {
    size_t idx_pos = 2 * i;
    size_t idx_vel = 2 * i + 1;


    double q    = state_interfaces_[idx_pos].get_value();
    double qdot = state_interfaces_[idx_vel].get_value();


    double qd    = (i < q_des.size())   ? q_des[i]  : q;
    double qdotd = (i < qd_des.size())  ? qd_des[i] : 0.0;


    double tau = kp_[i] * (qd - q) + kd_[i] * (qdotd - qdot);


    // Very simple fake-force effect: add Fx * gain to joint 0 torque
    if (i == 0 && fake_force_.size() == 6) {
      double Fx = fake_force_[0];
      tau += fake_force_gain_ * Fx;
    }


    command_interfaces_[i].set_value(tau);
  }


  return return_type::OK;
}


}  // namespace my_ur6_control


  PLUGINLIB_EXPORT_CLASS(my_ur5_control::ForceController, controller_interface::ControllerInterface)