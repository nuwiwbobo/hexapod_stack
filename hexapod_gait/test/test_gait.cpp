#include <gtest/gtest.h>
#include <hexapod_gait/gait_engine.hpp>
#include <cmath>

using namespace hexapod_gait;

class GaitTest : public ::testing::Test {
protected:
  void SetUp() override {
    GaitParams params;
    params.cycle_length = 50;
    params.lift_height = 0.0375;
    params.velocity_scaling = 0.15;
    params.low_pass_alpha = 0.05;
    engine = std::make_unique<GaitEngine>(params);
  }

  std::unique_ptr<GaitEngine> engine;
};

TEST_F(GaitTest, NoMotionProducesZeroFeet) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  auto result = engine->compute(cmd_vel);

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(result.foot[i].position.x, 0.0, 0.001);
    EXPECT_NEAR(result.foot[i].position.y, 0.0, 0.001);
    EXPECT_NEAR(result.foot[i].position.z, 0.0, 0.001);
  }
}

TEST_F(GaitTest, ForwardMotionLiftsSwingLegs) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  hexapod_msgs::msg::FeetPositions result;
  for (int step = 0; step < 25; ++step) {
    result = engine->compute(cmd_vel);
  }

  bool any_lifted = false;
  for (int i = 0; i < 6; ++i) {
    if (result.foot[i].position.z > 0.001) {
      any_lifted = true;
      break;
    }
  }
  EXPECT_TRUE(any_lifted) << "Forward motion should lift at least one swing leg";
}

TEST_F(GaitTest, LiftHeightRespected) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  double max_z = 0.0;
  for (int step = 0; step < 50; ++step) {
    auto result = engine->compute(cmd_vel);
    for (int i = 0; i < 6; ++i) {
      max_z = std::max(max_z, result.foot[i].position.z);
    }
  }

  EXPECT_LE(max_z, 0.0375 + 0.001);
  EXPECT_GE(max_z, 0.0375 - 0.001);
}
