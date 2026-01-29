// src/ur5e_path_planner_node.cpp
#include <memory>
#include <vector>
#include <string>
#include <cmath>  // clamp, M_PI

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class UR5ePathPlannerNode : public rclcpp::Node {
public:
  UR5ePathPlannerNode() : Node("ur5e_path_planner") {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&UR5ePathPlannerNode::jointStateCb, this, std::placeholders::_1));

    ee_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_goal", 10, std::bind(&UR5ePathPlannerNode::eeGoalCb, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/my_force_controller/trajectory", 10);  // Your controller topic

    traj_.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  }

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!have_joint_state_) {
      last_q_.resize(msg->position.size());
      have_joint_state_ = true;
      RCLCPP_INFO(get_logger(), "Got initial joints: %zu dof", msg->position.size());
    }
    last_q_ = msg->position;  // Update current
  }

  std::vector<double> computeIK(const geometry_msgs::msg::PoseStamped& goal, size_t dof) {
    std::vector<double> q_goal(dof, 0.0);
    if (dof < 6) return q_goal;

    // Scale pose → rough joints (UR5e/6 heuristic)
    q_goal[0] = std::clamp(goal.pose.position.x * 2.0, -M_PI, M_PI);  // shoulder_pan
    q_goal[1] = std::clamp(-M_PI/2.0 + goal.pose.position.z * 1.5, -M_PI, 0.0);  // lift + z
    q_goal[2] = std::clamp(goal.pose.position.y * 1.5, -M_PI/2, M_PI/2);  // elbow y
    q_goal[3] = -M_PI/2;  // wrist1 fixed
    q_goal[4] = 0.0;
    q_goal[5] = 0.0;

    // UR limits clamp [rad]: ±360° most, wrist3 free
    const double limits[6][2] = {
      {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI},
      {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI}, {-2*M_PI, 10*M_PI}
    };
    for (size_t i = 0; i < dof; ++i) {
      q_goal[i] = std::clamp(q_goal[i], limits[i][0], limits[i][1]);
    }

    RCLCPP_INFO(get_logger(), "IK from (%.2f,%.2f,%.2f) → q_goal=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                q_goal[0], q_goal[1], q_goal[2], q_goal[3], q_goal[4], q_goal[5]);
    return q_goal;
  }

  void planJointPath(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                     trajectory_msgs::msg::JointTrajectory& traj) {
    traj.points.clear();
    const size_t dof = q_start.size();
    const double duration = 4.0;  // 4s traj
    const int n_points = 20;

    for (int i = 0; i < n_points; ++i) {
      double t_frac = static_cast<double>(i) / (n_points - 1);
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(dof);
      pt.velocities.resize(dof, 0.0);  // Trapezoidal vel later if needed
      for (size_t j = 0; j < dof; ++j) {
        pt.positions[j] = q_start[j] + t_frac * (q_goal[j] - q_start[j]);
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(t_frac * duration);
      traj.points.push_back(pt);
    }
    RCLCPP_INFO(get_logger(), "Planned %d-pt traj over %.1fs", n_points, duration);
  }

  void eeGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!have_joint_state_) {
    RCLCPP_WARN(get_logger(), "No joints yet—wait for /joint_states");
    return;
  }

  std::vector<double> q_goal = computeIK(*msg, last_q_.size());
  planJointPath(last_q_, q_goal, traj_);

  // HOLD: Repeat final point (no drift)
  trajectory_msgs::msg::JointTrajectoryPoint hold_pt = traj_.points.back();
  double final_t = traj_.points.back().time_from_start.sec +
                   traj_.points.back().time_from_start.nanosec * 1e-9;
  for (int i = 1; i < 100; ++i) {
    double t_hold = final_t + i * 0.1;
    hold_pt.time_from_start.sec = static_cast<uint32_t>(t_hold);
    hold_pt.time_from_start.nanosec = static_cast<uint32_t>((t_hold - hold_pt.time_from_start.sec) * 1e9);
    hold_pt.velocities.assign(hold_pt.positions.size(), 0.0);
    traj_.points.push_back(hold_pt);
  }

  traj_pub_->publish(traj_);
  last_q_ = q_goal;
  RCLCPP_INFO(get_logger(), "Published %.1fs traj + hold to q_goal=[%.2f,%.2f,...]", final_t, q_goal[0], q_goal[1]);
}


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_goal_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  trajectory_msgs::msg::JointTrajectory traj_;
  std::vector<double> last_q_;
  bool have_joint_state_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5ePathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
