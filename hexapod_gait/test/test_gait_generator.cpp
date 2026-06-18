// =============================================================================
// Task 6: Unit tests for Gait Generator
// =============================================================================
// TDD approach: write these tests first, then implement gait_generator.cpp
//
// Test list:
//   1. InitialStance - all feet at neutral position initially
//   2. StepAdvancesCounter - after 48 steps, gait cycles back
//   3. SwingLegLifts - during swing phase, the swinging leg should lift
//   4. CorrectLegSwingSequence - each leg swings at correct step number
//   5. Reset - reset returns to initial state
//
// Run: colcon build --packages-select hexapod_gait && colcon test --packages-select hexapod_gait

#include <gtest/gtest.h>
#include <hexapod_gait/gait_generator.hpp>
#include <cmath>

using namespace hexapod_gait;

class GaitGeneratorTest : public ::testing::Test {
protected:
  void SetUp() override {
    GaitParams params;
    params.cycle_length = 48;
    params.lift_height = 0.03;
    params.step_time = 0.05;
    generator_ = std::make_unique<GaitGenerator>(params);
  }

  std::unique_ptr<GaitGenerator> generator_;
};

// TODO: Implement test - all feet should be at neutral position initially
TEST_F(GaitGeneratorTest, InitialStance) {
  auto feet = generator_->step(0.0, 0.0, 0.0);

  // All Z should be 0 (on the ground)
  for (const auto& foot : feet) {
    EXPECT_NEAR(foot.z, 0.0, 0.001);
  }
}

// TODO: Implement test - after 48 steps, gait should cycle back
TEST_F(GaitGeneratorTest, StepAdvancesCounter) {
  for (int i = 0; i < 48; ++i) {
    generator_->step(0.1, 0.0, 0.0);
  }

  // Step 48 should be same as step 0 (cycle wraps)
  auto feet_before = generator_->step(0.1, 0.0, 0.0);
  auto feet_after = feet_before;
  (void)feet_after;
}

// TODO: Implement test - during swing phase, the swinging leg should lift
TEST_F(GaitGeneratorTest, SwingLegLifts) {
  // Step 0: LF should be swinging (lifting)
  auto feet = generator_->step(0.1, 0.0, 0.0);

  // LF (index 0) should have positive Z (lifted)
  EXPECT_GT(feet[0].z, 0.0);

  // Other legs should be on ground (Z ≈ 0)
  for (int i = 1; i < 6; ++i) {
    EXPECT_NEAR(feet[i].z, 0.0, 0.001);
  }
}

// TODO: Implement test - verify each leg swings at correct step number
TEST_F(GaitGeneratorTest, CorrectLegSwingSequence) {
  int legs_per_cycle = 6;
  int steps_per_leg = 48 / legs_per_cycle;

  for (int leg = 0; leg < 6; ++leg) {
    for (int s = 0; s < steps_per_leg; ++s) {
      auto feet = generator_->step(0.1, 0.0, 0.0);

      if (s == steps_per_leg / 2) {
        // Mid-swing: this leg should be lifted
        EXPECT_GT(feet[leg].z, 0.0)
          << "Leg " << leg << " not lifted at step " << s;
      }
    }
  }
}

// TODO: Implement test - reset should return to initial state
TEST_F(GaitGeneratorTest, Reset) {
  generator_->step(0.1, 0.0, 0.0);
  generator_->step(0.1, 0.0, 0.0);
  generator_->reset();

  auto feet = generator_->step(0.0, 0.0, 0.0);
  for (const auto& foot : feet) {
    EXPECT_NEAR(foot.z, 0.0, 0.001);
  }
}
