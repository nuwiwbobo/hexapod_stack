#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <hexapod_control/control_node.hpp>

using namespace hexapod_control;

class ControlNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(ControlNodeTest, JointNamesAreCorrect) {
  auto names = ControlNode::getJointNames();
  ASSERT_EQ(names.size(), 18u);
  EXPECT_EQ(names[0], "coxa_joint_RR");
  EXPECT_EQ(names[17], "tibia_joint_LF");
}

TEST_F(ControlNodeTest, InitialBodyStateIsZero) {
  ControlNode node;
  auto body = node.getBodyOrientation();
  EXPECT_NEAR(body.roll, 0.0, 0.001);
  EXPECT_NEAR(body.pitch, 0.0, 0.001);
  EXPECT_NEAR(body.yaw, 0.0, 0.001);
}
