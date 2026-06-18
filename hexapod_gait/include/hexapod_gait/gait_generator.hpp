#ifndef HEXAPOD_GAIT__GAIT_GENERATOR_HPP_
#define HEXAPOD_GAIT__GAIT_GENERATOR_HPP_

// =============================================================================
// Task 6: Wave Gait Generator
// =============================================================================
// Generates foot trajectories for wave gait pattern.
//
// Wave gait: one leg swings at a time (unlike tripod's 3+3)
//   Step 0-7:   LF swings
//   Step 8-15:  RF swings
//   Step 16-23: LM swings
//   Step 24-31: RM swings
//   Step 32-39: LR swings
//   Step 40-47: RR swings
//
// Swing trajectory:
//   X/Y: cosine interpolation from start to end
//   Z: sine lift, max height = lift_height parameter
//
// Build order:
//   1. Implement gait_generator.hpp (this file)
//   2. Implement gait_generator.cpp
//   3. Write tests in test/test_gait_generator.cpp
//   4. Uncomment CMakeLists.txt targets

#include <array>
#include <cstdint>

namespace hexapod_gait {

// Leg indices (matches URDF order)
enum class Leg : uint8_t { LF = 0, RF, LM, RM, LR, RR };

// Gait parameters
struct GaitParams {
  double cycle_length;      // Steps per full cycle (48)
  double lift_height;       // Max foot lift in meters (0.03)
  double step_time;         // Seconds per step (0.05)
};

// Foot command for one leg
struct FootCommand {
  double x, y, z;
};

class GaitGenerator {
public:
  explicit GaitGenerator(const GaitParams& params);

  // =========================================================================
  // TODO: Implement these methods
  // =========================================================================

  // Advance gait by one step, return foot positions for all 6 legs
  // vx, vy: linear velocity (m/s)
  // angular: angular velocity (rad/s)
  std::array<FootCommand, 6> step(double vx, double vy, double angular);

  // Reset gait to initial stance
  void reset();

private:
  GaitParams params_;
  int step_counter_;
  std::array<FootCommand, 6> default_stance_;
};

}  // namespace hexapod_gait

#endif  // HEXAPOD_GAIT__GAIT_GENERATOR_HPP_
