// =============================================================================
// gait_generator.cpp — Wave gait generator
// =============================================================================
// Wave gait: one leg swings at a time.
//   Step 0-7:   LF swings
//   Step 8-15:  RF swings
//   Step 16-23: LM swings
//   Step 24-31: RM swings
//   Step 32-39: LR swings
//   Step 40-47: RR swings
//
// Swing trajectory:
//   Z: sine lift — lift_height * sin(π * (t+1) / swing_steps)
//   XY: linear interpolation from start to end based on velocity

#include <hexapod_gait/gait_generator.hpp>

#include <cmath>

namespace hexapod_gait {

GaitGenerator::GaitGenerator(const GaitParams& params)
: params_(params),
  step_counter_(0)
{
  default_stance_.fill({0.0, 0.0, 0.0});
}

std::array<FootCommand, 6> GaitGenerator::step(double vx, double vy, double angular)
{
  const int swing_steps = static_cast<int>(params_.cycle_length) / 6;
  const int leg_index = step_counter_ / swing_steps;
  const int step_in_swing = step_counter_ % swing_steps;

  std::array<FootCommand, 6> feet = default_stance_;

  // Per-step displacement
  const double dx = vx * params_.step_time;
  const double dy = vy * params_.step_time;

  const double speed = std::sqrt(vx * vx + vy * vy);
  const bool moving = (speed > 1e-6) || (std::abs(angular) > 1e-6);

  for (int i = 0; i < 6; ++i) {
    if (i == leg_index && moving) {
      // Swing phase: lift foot and move forward
      const double t = static_cast<double>(step_in_swing);
      const double progress = (t + 1.0) / static_cast<double>(swing_steps);
      feet[i].z = params_.lift_height * std::sin(M_PI * progress);
      feet[i].x = dx * progress;
      feet[i].y = dy * progress;
    } else {
      // Stance phase or not moving: foot on ground
      feet[i].z = 0.0;
      feet[i].x = 0.0;
      feet[i].y = 0.0;
    }
  }

  step_counter_ = (step_counter_ + 1) % static_cast<int>(params_.cycle_length);
  return feet;
}

void GaitGenerator::reset()
{
  step_counter_ = 0;
}

}  // namespace hexapod_gait
