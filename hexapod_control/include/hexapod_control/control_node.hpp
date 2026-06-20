#ifndef HEXAPOD_CONTROL__CONTROL_NODE_HPP_
#define HEXAPOD_CONTROL__CONTROL_NODE_HPP_

// =============================================================================
// Control Orchestrator Node
// =============================================================================
// Orchestrates: cmd_vel → gait → IK → /joint_targets
//
// Data flow:
//   /cmd_vel (Twist)
//        │
//        ▼
//   GaitGenerator::step()  →  FootCommand[6]
//        │
//        ▼
//   IkSolver::solve() × 6  →  LegJoints[6]  (18 joint angles)
//        │
//        ▼
//   publish /joint_targets (JointState, 18 joints)
//
// No ServoDriver — hardware handled by servo_node via /joint_targets topic.

#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hexapod_ik/ik_solver.hpp>
#include <hexapod_gait/gait_generator.hpp>

namespace hexapod_control {

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlLoop();

  // Gait generator
  hexapod_gait::GaitGenerator gait_generator_;

  // 6 IK solvers (one per leg, same params)
  std::array<hexapod_ik::IkSolver, 6> ik_solvers_;

  // Joint name mapping: [leg_index][joint_index(0=coxa,1=femur,2=tibia)]
  std::array<std::array<std::string, 3>, 6> joint_names_;

  // Flat list of all 18 joint names (for JointState message)
  std::vector<std::string> all_joint_names_;

  // cmd_vel storage
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_angular_{0.0};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace hexapod_control

#endif  // HEXAPOD_CONTROL__CONTROL_NODE_HPP_
