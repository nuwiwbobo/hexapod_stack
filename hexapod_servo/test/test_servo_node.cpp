// =============================================================================
// Task 3: Integration test for ServoNode
// =============================================================================
// Tests the ROS 2 node wrapper around ServoDriver.
//
// Prerequisites:
//   1. Complete Task 1 (servo_driver.hpp/cpp)
//   2. Uncomment ROS deps in CMakeLists.txt and package.xml
//   3. Implement servo_node.hpp/cpp
//
// Test:
//   - Creates ServoNode
//   - Subscribes to /joint_states
//   - Spins for 100ms
//   - Verifies at least one message received
//
// Run: colcon test --packages-select hexapod_servo

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_servo/servo_node.hpp>

class ServoNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

TEST_F(ServoNodeTest, PublishesJointState) {
  auto node = std::make_shared<hexapod_servo::ServoNode>();

  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&](sensor_msgs::msg::JointState::SharedPtr msg) {
      received = msg;
    }
  );

  // Spin for 100ms to allow timer callback to fire
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_NE(received, nullptr);
}
