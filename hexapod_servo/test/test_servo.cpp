#include <gtest/gtest.h>
#include <hexapod_servo/servo_driver.hpp>
#include <cmath>

using namespace hexapod_servo;

class ServoTest : public ::testing::Test {
protected:
  void SetUp() override {
    ServoParams params;
    params.servo_count = 18;
    params.baud_rate = 1000000;
    params.protocol_version = 1.0;
    params.torque_enable_reg = 24;
    params.present_position_reg = 36;
    params.goal_position_reg = 30;

    ServoConfig servo;
    servo.id = 1;
    servo.ticks = 1024;
    servo.center = 512;
    servo.max_radians = 5.236;
    servo.sign = 1;
    servo.offset = 0.0;
    params.servos.push_back(servo);
    params.joint_names.push_back("coxa_joint_LF");

    driver = std::make_unique<ServoDriver>(params);
  }

  ServoParams params;
  std::unique_ptr<ServoDriver> driver;
};

TEST_F(ServoTest, RadianToTickConversion) {
  uint16_t tick = driver->radianToTick(0.0, 0);
  EXPECT_EQ(tick, 512);

  uint16_t tick_pos = driver->radianToTick(0.5, 0);
  EXPECT_GT(tick_pos, 512);

  uint16_t tick_neg = driver->radianToTick(-0.5, 0);
  EXPECT_LT(tick_neg, 512);
}

TEST_F(ServoTest, TickToRadianConversion) {
  double rad = driver->tickToRadian(512, 0);
  EXPECT_NEAR(rad, 0.0, 0.01);

  double rad_pos = driver->tickToRadian(600, 0);
  EXPECT_GT(rad_pos, 0.0);

  double rad_neg = driver->tickToRadian(400, 0);
  EXPECT_LT(rad_neg, 0.0);
}

TEST_F(ServoTest, BidirectionalConversion) {
  double original = 0.3142;
  uint16_t tick = driver->radianToTick(original, 0);
  double recovered = driver->tickToRadian(tick, 0);
  EXPECT_NEAR(original, recovered, 0.01);
}
