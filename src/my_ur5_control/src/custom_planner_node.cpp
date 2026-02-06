// src/ur5e_path_planner_node.cpp

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>  // For matrix ops

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class UR5ePathPlannerNode : public rclcpp::Node {
public:
  UR5ePathPlannerNode() : Node("ur5e_path_planner") {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&UR5ePathPlannerNode::jointStateCb, this, std::placeholders::_1));

    ee_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_goal", 10,
      std::bind(&UR5ePathPlannerNode::eeGoalCb, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/my_force_controller/trajectory", 10);

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
    last_q_ = msg->position;
  }

  std::vector<double> computeIK(const geometry_msgs::msg::PoseStamped& goal, size_t dof) {
    std::vector<double> q_goal(dof, 0.0);
    if (dof < 6) return q_goal;

    // --- UR5e DH params (meters) ---
    const double d1 = 0.1625;
    const double a2 = -0.425;
    const double a3 = -0.3922;
    const double d4 = 0.1333;
    const double d5 = 0.0997;
    const double d6 = 0.0996;

    // --- Extract goal position and orientation ---
    double px = goal.pose.position.x;
    double py = goal.pose.position.y;
    double pz = goal.pose.position.z;

    // Orientation as rotation matrix
    double qw = goal.pose.orientation.w;
    double qx = goal.pose.orientation.x;
    double qy = goal.pose.orientation.y;
    double qz = goal.pose.orientation.z;

    Eigen::Matrix3d R;
    R(0,0) = 1 - 2*(qy*qy + qz*qz);
    R(0,1) = 2*(qx*qy - qz*qw);
    R(0,2) = 2*(qx*qz + qy*qw);
    R(1,0) = 2*(qx*qy + qz*qw);
    R(1,1) = 1 - 2*(qx*qx + qz*qz);
    R(1,2) = 2*(qy*qz - qx*qw);
    R(2,0) = 2*(qx*qz - qy*qw);
    R(2,1) = 2*(qy*qz + qx*qw);
    R(2,2) = 1 - 2*(qx*qx + qy*qy);

    // --- Wrist center ---
    Eigen::Vector3d pw(px, py, pz);
    Eigen::Vector3d z_axis = R.col(2);  // End-effector z-axis
    Eigen::Vector3d wc = pw - d6 * z_axis;

    // --- Theta1 ---
    double theta1 = atan2(wc.y(), wc.x());

    // --- Theta3 (elbow) ---
    double r = sqrt(wc.x()*wc.x() + wc.y()*wc.y());
    double s = wc.z() - d1;
    double D = (r*r + s*s - a2*a2 - a3*a3)/(2*a2*a3);
    D = std::clamp(D, -1.0, 1.0);
    double theta3 = atan2(-sqrt(1 - D*D), D);  // elbow-down

    // --- Theta2 (shoulder) ---
    double phi1 = atan2(s, r);
    double phi2 = atan2(a3 * sin(theta3), a2 + a3*cos(theta3));
    double theta2 = phi1 - phi2;

    // --- Theta4,5,6 (wrist) ---
    double c1 = cos(theta1), s1 = sin(theta1);
    double c2 = cos(theta2), s2 = sin(theta2);
    double c3 = cos(theta3), s3 = sin(theta3);

    Eigen::Matrix3d R0_3;
    R0_3 << c1*(c2*c3 - s2*s3), -c1*(c2*s3 + s2*c3), s1,
            s1*(c2*c3 - s2*s3), -s1*(c2*s3 + s2*c3), -c1,
            s2*c3 + c2*s3, -(s2*s3 - c2*c3), 0;

    Eigen::Matrix3d R3_6 = R0_3.transpose() * R;

    double theta5 = atan2(sqrt(R3_6(0,2)*R3_6(0,2) + R3_6(1,2)*R3_6(1,2)), R3_6(2,2));
    double theta4 = atan2(R3_6(1,2), R3_6(0,2));
    double theta6 = atan2(R3_6(2,1), -R3_6(2,0));

    q_goal[0] = theta1;
    q_goal[1] = theta2;
    q_goal[2] = theta3;
    q_goal[3] = theta4;
    q_goal[4] = theta5;
    q_goal[5] = theta6;

    // --- Clamp to joint limits ---
    const double limits[6][2] = {
        {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI},
        {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI}, {-2*M_PI, 2*M_PI}
    };
    for (size_t i = 0; i < dof; ++i) {
        q_goal[i] = std::clamp(q_goal[i], limits[i][0], limits[i][1]);
    }

    RCLCPP_INFO(get_logger(),
        "Analytical IK: px=%.2f py=%.2f pz=%.2f → q=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
        px, py, pz,
        q_goal[0], q_goal[1], q_goal[2],
        q_goal[3], q_goal[4], q_goal[5]);

    return q_goal;
  }

  void planJointPath(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                     trajectory_msgs::msg::JointTrajectory& traj) {
    traj.points.clear();
    const size_t dof = q_start.size();
    const double duration = 4.0;
    const int n_points = 20;

    for (int i = 0; i < n_points; ++i) {
      double t_frac = static_cast<double>(i) / (n_points - 1);
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(dof);
      pt.velocities.resize(dof, 0.0);
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

    // HOLD: Repeat final point
    trajectory_msgs::msg::JointTrajectoryPoint hold_pt = traj_.points.back();
    double final_t = traj_.points.back().time_from_start.sec +
                     traj_.points.back().time_from_start.nanosec * 1e-9;
    for (int i = 1; i < 100; ++i) {
      double t_hold = final_t + i * 0.1;
      hold_pt.time_from_start.sec = static_cast<uint32_t>(t_hold);
      hold_pt.time_from_start.nanosec =
          static_cast<uint32_t>((t_hold - hold_pt.time_from_start.sec) * 1e9);
      hold_pt.velocities.assign(hold_pt.positions.size(), 0.0);
      traj_.points.push_back(hold_pt);
    }

    traj_pub_->publish(traj_);
    last_q_ = q_goal;
    RCLCPP_INFO(get_logger(),
                "Published %.1fs traj + hold to q_goal=[%.2f,%.2f,...]",
                final_t, q_goal[0], q_goal[1]);
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
