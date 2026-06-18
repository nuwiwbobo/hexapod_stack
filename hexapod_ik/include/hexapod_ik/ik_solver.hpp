#ifndef HEXAPOD_IK__IK_SOLVER_HPP_
#define HEXAPOD_IK__IK_SOLVER_HPP_

// =============================================================================
// Task 5: Analytical IK Solver (law of cosines)
// =============================================================================
// Solves inverse kinematics for a single 3-DOF leg.
//
// Algorithm (ported from ROS 1):
//   1. Coxa angle: atan2(x, y) — direction to foot
//   2. Project to XY plane, subtract coxa link length
//   3. Triangle solve: law of cosines for femur/tibia angles
//   4. Reachability check: if distance > femur + tibia → UNREACHABLE
//
// Reference: /home/wawabobo/ROS-package/hexapod_controller/include/ik.h
//
// Build order:
//   1. Implement ik_solver.hpp (this file)
//   2. Implement ik_solver.cpp
//   3. Write tests in test/test_ik_solver.cpp
//   4. Uncomment CMakeLists.txt targets

#include <cmath>

namespace hexapod_ik {

// Physical dimensions of one leg
struct LegParams {
  double coxa_length;    // 0.044 m (coxa link)
  double femur_length;   // 0.0545 m (upper leg)
  double tibia_length;   // 0.1019 m (lower leg)
};

// Foot position in body frame
struct FootPosition {
  double x, y, z;
};

// Joint angles for one leg
struct LegJoints {
  double coxa, femur, tibia;
};

// IK solve result status
enum class IKStatus { SUCCESS, UNREACHABLE, ERROR };

// Complete IK result
struct IKResult {
  IKStatus status;
  LegJoints joints;
};

class IkSolver {
public:
  explicit IkSolver(const LegParams& params);

  // =========================================================================
  // TODO: Implement these methods
  // =========================================================================

  // Inverse kinematics: foot position → joint angles
  // Returns UNREACHABLE if foot is beyond max reach
  IKResult solve(const FootPosition& foot) const;

  // Forward kinematics: joint angles → foot position
  // Used for verification/testing
  FootPosition forward(const LegJoints& joints) const;

private:
  LegParams params_;
};

}  // namespace hexapod_ik

#endif  // HEXAPOD_IK__IK_SOLVER_HPP_
