#ifndef HEXAPOD_CONTROL__CONTROL_NODE_HPP_
#define HEXAPOD_CONTROL__CONTROL_NODE_HPP_

// =============================================================================
// Task 7: Control Orchestrator Node
// =============================================================================
// Orchestrates the full pipeline: cmd_vel → gait → IK → servo
//
// Data flow (single process, intra-process composable):
//   cmd_vel (Twist)
//        │
//        ▼
//   GaitGenerator::step()  →  FootCommand[6]
//        │
//        ▼
//   IkSolver::solve()  →  LegJoints[6]
//        │
//        ▼
//   ServoDriver::setGoalPositions()  →  hardware
//
// Topics:
//   /cmd_vel (Twist) — input from teleop
//   /joint_states (JointState) — output to ros2_control
//
// Build order:
//   1. Complete Tasks 1-6 (servo, msgs, ik, gait)
//   2. Implement control_node.hpp/cpp
//   3. Write integration test in test/test_control_node.cpp
//   4. Uncomment CMakeLists.txt targets

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hexapod_servo/servo_driver.hpp>
#include <hexapod_ik/ik_solver.hpp>
#include <hexapod_gait/gait_generator.hpp>

namespace hexapod_control {

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlLoop();

  hexapod_servo::ServoDriver servo_driver_;
  hexapod_ik::IkSolver ik_solver_;
  hexapod_gait::GaitGenerator gait_generator_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double cmd_vx_, cmd_vy_, cmd_angular_;
};

}  // namespace hexapod_control

#endif  // HEXAPOD_CONTROL__CONTROL_NODE_HPP_
