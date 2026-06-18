// =============================================================================
// Task 9: Integration test for ControlNode
// =============================================================================
// Tests the full pipeline: cmd_vel → gait → IK → servo
//
// Prerequisites:
//   1. Complete Tasks 1-6 (servo, msgs, ik, gait)
//   2. Implement control_node.hpp/cpp
//
// Test:
//   - Creates ControlNode
//   - Publishes cmd_vel
//   - Subscribes to /joint_states
//   - Verifies joint states are published
//
// Run: colcon test --packages-select hexapod_control

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_control/control_node.hpp>

class ControlNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

TEST_F(ControlNodeTest, ReceivesCmdVel) {
  auto node = std::make_shared<hexapod_control::ControlNode>();

  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&](sensor_msgs::msg::JointState::SharedPtr msg) {
      received = msg;
    }
  );

  // Publish a cmd_vel
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = 0.1;
  pub->publish(msg);

  // Spin to process callbacks
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_NE(received, nullptr);
  if (received) {
    EXPECT_GT(received->position.size(), 0u);
  }
}
