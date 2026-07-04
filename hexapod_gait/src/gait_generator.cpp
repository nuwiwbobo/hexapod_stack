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
  default_stance_.fill({0.0, 0.0, 0.0});              // Default foot positions (x, y, z) in the stance phase
}

std::array<FootCommand, 6> GaitGenerator::step(double vx, double vy, double angular)  // vx: forward velocity, vy: lateral velocity, angular: rotational velocity
{
  const int swing_steps = static_cast<int>(params_.cycle_length) / 6;                 // Each leg swings for 1/6 of the cycle
  const int leg_index = step_counter_ / swing_steps;                                  // Determine which leg is swinging based on the current step
  const int step_in_swing = step_counter_ % swing_steps;                              // Step index within the current leg's swing phase

  std::array<FootCommand, 6> feet = default_stance_;                                  // Start with default stance positions

  // Per-step displacement
  const double dx = vx * params_.step_time;                                           // Forward displacement based on forward velocity
  const double dy = vy * params_.step_time;                                           // Lateral displacement based on lateral velocity

  const double speed = std::sqrt(vx * vx + vy * vy);                                  // Calculate the overall speed from the forward and lateral velocities
  const bool moving = (speed > 1e-6) || (std::abs(angular) > 1e-6);                   // Determine if the robot is moving based on the speed and angular velocity

  for (int i = 0; i < 6; ++i) {                                                       // For each leg, determine if it's in the swing phase or stance phase
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
