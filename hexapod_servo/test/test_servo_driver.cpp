// =============================================================================
// Task 1: Unit tests for ServoDriver
// =============================================================================
// TDD approach: write these tests first, then implement servo_driver.cpp
//
// Test list:
//   1. RadianToTickCenter   - 0.0 radians → center tick (512)
//   2. RadianToTickPositive - M_PI/2 → ~768  (512 + 256)
//   3. RadianToTickNegative - -M_PI/2 → ~256 (512 - 256)
//   4. TickToRadianCenter   - center tick → 0.0 radians
//   5. RoundTrip            - radianToTick then tickToRadian recovers original
//   6. BoundaryClamping     - extreme values stay in [0, ticks]
//   7. SignHandling         - sign=1 vs sign=-1 produce symmetric results
//
// NOTE on max_radians:
//   The AX-18A physical range is 300° (5.236 rad), but we configure
//   max_radians = 2*M_PI so that M_PI/2 maps cleanly to 1/4 of the tick
//   range (512 + 256 = 768).  If you prefer physical accuracy, set
//   max_radians = 5.236 and adjust the EXPECT_NEAR targets to ~819 / ~205.
//
// Run: colcon build --packages-select hexapod_servo
//      colcon test  --packages-select hexapod_servo

#include <gtest/gtest.h>
#include <hexapod_servo/servo_driver.hpp>
#include <cmath>

using namespace hexapod_servo;

class ServoDriverTest : public ::testing::Test {
protected:
  void SetUp() override {
    ServoConfig servo;
    servo.id         = 1;
    servo.type       = "AX-18A";
    servo.ticks      = 1024;
    servo.center     = 512;
    servo.max_radians = 2.0 * M_PI;  // Full 360° convention: M_PI/2 → 768
    servo.offset     = 0.0;
    servo.sign       = 1;
    servos_.push_back(servo);
    driver_ = std::make_unique<ServoDriver>(servos_);
  }

  std::vector<ServoConfig> servos_;
  std::unique_ptr<ServoDriver> driver_;
};

// 0.0 radians should map to center tick (512)
TEST_F(ServoDriverTest, RadianToTickCenter) {
  uint16_t tick = driver_->radianToTick(0.0, 0);
  EXPECT_EQ(tick, 512);
}

// M_PI/2 should map to ~768 (512 + 256)
TEST_F(ServoDriverTest, RadianToTickPositive) {
  uint16_t tick = driver_->radianToTick(M_PI / 2.0, 0);
  EXPECT_NEAR(tick, 768, 2);
}

// -M_PI/2 should map to ~256 (512 - 256)
TEST_F(ServoDriverTest, RadianToTickNegative) {
  uint16_t tick = driver_->radianToTick(-M_PI / 2.0, 0);
  EXPECT_NEAR(tick, 256, 2);
}

// center tick (512) should map to 0.0 radians
TEST_F(ServoDriverTest, TickToRadianCenter) {
  double radian = driver_->tickToRadian(512, 0);
  EXPECT_NEAR(radian, 0.0, 0.001);
}

// Round-trip conversion should recover original value within quantization tolerance
TEST_F(ServoDriverTest, RoundTrip) {
  double original = M_PI / 4.0;
  uint16_t tick   = driver_->radianToTick(original, 0);
  double recovered = driver_->tickToRadian(tick, 0);
  EXPECT_NEAR(original, recovered, 0.01);
}

// Extreme values should be clamped to [0, ticks]
TEST_F(ServoDriverTest, BoundaryClamping) {
  uint16_t tick_high = driver_->radianToTick(100.0, 0);
  EXPECT_LE(tick_high, 1024);

  uint16_t tick_low = driver_->radianToTick(-100.0, 0);
  EXPECT_GE(tick_low, 0);
}

// sign=1 vs sign=-1 should produce results symmetric around center
TEST_F(ServoDriverTest, SignHandling) {
  ServoConfig servo_neg;
  servo_neg.id          = 2;
  servo_neg.type        = "AX-18A";
  servo_neg.ticks       = 1024;
  servo_neg.center      = 512;
  servo_neg.max_radians = 2.0 * M_PI;
  servo_neg.offset      = 0.0;
  servo_neg.sign        = -1;
  std::vector<ServoConfig> servos_neg = {servo_neg};
  ServoDriver driver_neg(servos_neg);

  uint16_t tick_pos = driver_->radianToTick(M_PI / 4.0, 0);
  uint16_t tick_neg = driver_neg.radianToTick(M_PI / 4.0, 0);
  EXPECT_NE(tick_pos, tick_neg);
  EXPECT_EQ(tick_pos - 512, 512 - tick_neg);  // Symmetric around center
}
