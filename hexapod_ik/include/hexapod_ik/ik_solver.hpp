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
struct FootPosition {    // x, y, z; // Position of the foot in 3D space relative to the body frame
  double x, y, z;        // x: forward/backward, y: left/right, z: up/down
};

// Joint angles for one leg
struct LegJoints {           // coxa, femur, tibia; // Angles of the coxa, femur, and tibia joints in radians
  double coxa, femur, tibia; // Coxa: rotation around vertical axis, Femur: rotation around horizontal axis, Tibia: rotation around horizontal axis
};

// IK solve result status
enum class IKStatus { SUCCESS, UNREACHABLE, ERROR };  // SUCCESS: valid solution found, UNREACHABLE: foot position is beyond leg's reach, ERROR: invalid input or computation error

// Complete IK result
struct IKResult {                                     // Struct to hold the result of the inverse kinematics computation, including the status and the computed joint angles
  IKStatus status;                                    // Status of the IK solution (e.g., SUCCESS, UNREACHABLE, ERROR)
  LegJoints joints;                                   // The computed joint angles for the coxa, femur, and tibia if the status is SUCCESS; otherwise, these may be set to NaN or ignored                     
};

class IkSolver {
public:
  explicit IkSolver(const LegParams& params);         // Constructor that takes the physical parameters of the leg to initialize the IK solver

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
