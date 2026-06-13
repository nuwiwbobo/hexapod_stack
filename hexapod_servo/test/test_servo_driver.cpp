#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "hexapod_servo/servo_driver.hpp"

using hexapod_servo::ServoConfig;
using hexapod_servo::ServoDriver;

static ServoConfig makeAX18A(int id, double offset = 0.0, int sign = 1) {
  return ServoConfig{id, "AX-18A", 1024, 512, M_PI, offset, sign};
}

class ServoDriverTest : public ::testing::Test {
protected:
  void SetUp() override {
    servos_ = {makeAX18A(1), makeAX18A(2, 0.1, -1)};
    driver_ = std::make_unique<ServoDriver>(servos_);
  }
  std::vector<ServoConfig> servos_;
  std::unique_ptr<ServoDriver> driver_;
};

TEST_F(ServoDriverTest, RadianToTickCenter) {
  EXPECT_EQ(driver_->radianToTick(0.0, 0), 512);
}

TEST_F(ServoDriverTest, RadianToTickPositive) {
  uint16_t tick = driver_->radianToTick(M_PI / 2.0, 0);
  EXPECT_NEAR(tick, 768, 2);
}

TEST_F(ServoDriverTest, RadianToTickNegative) {
  uint16_t tick = driver_->radianToTick(-M_PI / 2.0, 0);
  EXPECT_NEAR(tick, 256, 2);
}

TEST_F(ServoDriverTest, TickToRadianCenter) {
  EXPECT_NEAR(driver_->tickToRadian(512, 0), 0.0, 1e-6);
}

TEST_F(ServoDriverTest, RoundTrip) {
  double original = M_PI / 3.0;
  uint16_t tick = driver_->radianToTick(original, 0);
  double recovered = driver_->tickToRadian(tick, 0);
  EXPECT_NEAR(original, recovered, 0.01);
}

TEST_F(ServoDriverTest, BoundaryClamping) {
  uint16_t low = driver_->radianToTick(-100.0, 0);
  uint16_t high = driver_->radianToTick(100.0, 0);
  EXPECT_GE(low, 0u);
  EXPECT_LE(high, 1024u);
}

TEST_F(ServoDriverTest, SignHandling) {
  ServoDriver d({makeAX18A(1, 0.0, 1), makeAX18A(2, 0.0, -1)});
  uint16_t t1 = d.radianToTick(M_PI / 4.0, 0);
  uint16_t t2 = d.radianToTick(M_PI / 4.0, 1);
  EXPECT_NEAR(t1 + t2, 2 * 512u, 2);
}
