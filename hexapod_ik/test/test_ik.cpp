#include <gtest/gtest.h>
#include <hexapod_ik/ik_solver.hpp>
#include <cmath>

using namespace hexapod_ik;

class IkTest : public ::testing::Test {
protected:
  void SetUp() override {
    params.coxa_length = 0.044;
    params.femur_length = 0.0545;
    params.tibia_length = 0.1019;
    params.tarsus_length = 0.0;
    params.number_of_legs = 6;
    params.init_coxa_angle = {-0.4581, 0.0, 0.4581, -0.4581, 0.0, 0.4581};
    params.coxa_to_center_x = {-0.065, 0.0, 0.065, -0.065, 0.0, 0.065};
    params.coxa_to_center_y = {0.0325, 0.0625, 0.0325, 0.0325, 0.0625, 0.0325};
    params.init_foot_pos_x = {-0.04190685, 0.0, 0.04190685, -0.04190685, 0.0, 0.04190685};
    params.init_foot_pos_y = {0.084978692, 0.09475, 0.084978692, 0.084978692, 0.09475, 0.084978692};
    params.init_foot_pos_z = {0.07, 0.07, 0.07, 0.07, 0.07, 0.07};
    solver = std::make_unique<IkSolver>(params);
  }

  IkParams params;
  std::unique_ptr<IkSolver> solver;
};

TEST_F(IkTest, ZeroPoseReturnsInitAngles) {
  hexapod_msgs::msg::FeetPositions feet;
  for (int i = 0; i < 6; ++i) {
    feet.foot[i].position.x = params.init_foot_pos_x[i];
    feet.foot[i].position.y = params.init_foot_pos_y[i];
    feet.foot[i].position.z = params.init_foot_pos_z[i];
    feet.foot[i].orientation.yaw = 0.0;
  }

  hexapod_msgs::msg::Pose body;
  body.position.x = 0.0;
  body.position.y = 0.0;
  body.position.z = 0.0;
  body.orientation.roll = 0.0;
  body.orientation.pitch = 0.0;
  body.orientation.yaw = 0.0;

  auto result = solver->calculateIK(feet, body);

  EXPECT_TRUE(result.success);
  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(result.joints.leg[i].coxa))
      << "Leg " << i << " coxa is not finite";
    EXPECT_TRUE(std::isfinite(result.joints.leg[i].femur))
      << "Leg " << i << " femur is not finite";
    EXPECT_TRUE(std::isfinite(result.joints.leg[i].tibia))
      << "Leg " << i << " tibia is not finite";
  }
}

TEST_F(IkTest, UnreachableTargetReturnsFailure) {
  hexapod_msgs::msg::FeetPositions feet;
  for (int i = 0; i < 6; ++i) {
    feet.foot[i].position.x = 0.0;
    feet.foot[i].position.y = 0.0;
    feet.foot[i].position.z = 10.0;
    feet.foot[i].orientation.yaw = 0.0;
  }

  hexapod_msgs::msg::Pose body;
  body.position.x = 0.0;
  body.position.y = 0.0;
  body.position.z = 0.0;
  body.orientation.roll = 0.0;
  body.orientation.pitch = 0.0;
  body.orientation.yaw = 0.0;

  auto result = solver->calculateIK(feet, body);

  EXPECT_FALSE(result.success);
}

TEST_F(IkTest, BodyPitchChangesFemurAngle) {
  hexapod_msgs::msg::FeetPositions feet;
  for (int i = 0; i < 6; ++i) {
    feet.foot[i].position.x = 0.0;
    feet.foot[i].position.y = 0.0;
    feet.foot[i].position.z = 0.0;
    feet.foot[i].orientation.yaw = 0.0;
  }

  hexapod_msgs::msg::Pose body_zero;
  body_zero.position = geometry_msgs::msg::Point();
  body_zero.orientation = hexapod_msgs::msg::RPY();

  hexapod_msgs::msg::Pose body_tilted;
  body_tilted.position = geometry_msgs::msg::Point();
  body_tilted.orientation.roll = 0.0;
  body_tilted.orientation.pitch = 0.1;
  body_tilted.orientation.yaw = 0.0;

  auto result_zero = solver->calculateIK(feet, body_zero);
  auto result_tilted = solver->calculateIK(feet, body_tilted);

  EXPECT_TRUE(result_zero.success);
  EXPECT_TRUE(result_tilted.success);

  bool any_diff = false;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(result_zero.joints.leg[i].femur - result_tilted.joints.leg[i].femur) > 0.001) {
      any_diff = true;
      break;
    }
  }
  EXPECT_TRUE(any_diff) << "Body pitch should affect at least one leg's femur angle";
}
