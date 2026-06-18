#ifndef HEXAPOD_SERVO__SERVO_NODE_HPP_
#define HEXAPOD_SERVO__SERVO_NODE_HPP_

// =============================================================================
// Task 3: ROS 2 Node Wrapper for ServoDriver
// =============================================================================
// This node wraps the bare ServoDriver class with ROS 2 pub/sub.
//
// Behavior:
//   - Subscribes to /joint_targets (sensor_msgs/JointState)
//   - Publishes /joint_states (sensor_msgs/JointState) on a timer (10 Hz)
//   - Parameters: port, baud_rate (declared but not used until hardware)
//
// Build order:
//   1. Complete Task 1 (servo_driver.hpp/cpp)
//   2. Uncomment ROS deps in CMakeLists.txt and package.xml
//   3. Implement this node
//   4. Add integration test in test/test_servo_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hexapod_servo/servo_driver.hpp>

namespace hexapod_servo {

class ServoNode : public rclcpp::Node {
public:
  ServoNode();

private:
  void jointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  ServoDriver driver_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_NODE_HPP_
