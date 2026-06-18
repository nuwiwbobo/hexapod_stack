// =============================================================================
// ik_solver.cpp — Analytical IK/FK for a single 3-DOF hexapod leg
// =============================================================================
// No ROS dependency — pure C++ / cmath.
//
// Angle conventions
// -----------------
//   coxa  : yaw in XY plane; 0 = forward (+Y); positive = toward +X (right)
//           atan2(x, y) — consistent with typical ROS1 hexapod drivers
//   femur : pitch in the leg plane; 0 = horizontal; negative = pointing DOWN
//           (normal walking stance has femur < 0)
//   tibia : pitch relative to femur direction; 0 = straight;
//           positive = knee bends upward/forward (away from ground)
//           world tibia angle = femur + tibia
//
// FK summary
// ----------
//   1. Coxa tip   = (coxa_l * sin(ca),  coxa_l * cos(ca),  0)
//   2. Femur end  = coxa_tip + (femur_l * cos(fa), 0, femur_l * sin(fa))  [radial, z]
//   3. Foot       = femur_end + (tibia_l * cos(fa+ta), 0, tibia_l * sin(fa+ta))
//   4. Expand radial component along coxa direction in XY
//
// IK summary (law of cosines, elbow-down / knee-up solution)
// ----------------------------------------------------------
//   1. ca    = atan2(fx, fy)
//   2. r     = sqrt(fx²+fy²) − coxa_l          [reach from coxa joint]
//   3. d     = sqrt(r²+fz²)                     [coxa-tip to foot distance]
//   4. fa    = atan2(fz, r) − acos((fl²+d²−tl²)/(2·fl·d))
//   5. ta    = π − acos((fl²+tl²−d²)/(2·fl·tl))

#include <hexapod_ik/ik_solver.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hexapod_ik {

IkSolver::IkSolver(const LegParams& params)
: params_(params)
{
  if (params_.coxa_length  <= 0.0 ||
      params_.femur_length <= 0.0 ||
      params_.tibia_length <= 0.0)
  {
    throw std::invalid_argument("All leg link lengths must be positive");
  }
}

// ---------------------------------------------------------------------------
// Forward kinematics: joint angles → foot position
// ---------------------------------------------------------------------------
FootPosition IkSolver::forward(const LegJoints& j) const
{
  const double cl = params_.coxa_length;
  const double fl = params_.femur_length;
  const double tl = params_.tibia_length;

  // Radial reach in the leg plane (outward from body)
  const double world_tibia = j.femur + j.tibia;
  const double reach = fl * std::cos(j.femur) + tl * std::cos(world_tibia);
  const double fz    = fl * std::sin(j.femur) + tl * std::sin(world_tibia);

  // Project radial reach into XY using coxa direction
  const double sin_ca = std::sin(j.coxa);
  const double cos_ca = std::cos(j.coxa);

  FootPosition foot;
  foot.x = (cl + reach) * sin_ca;
  foot.y = (cl + reach) * cos_ca;
  foot.z = fz;
  return foot;
}

// ---------------------------------------------------------------------------
// Inverse kinematics: foot position → joint angles
// ---------------------------------------------------------------------------
IKResult IkSolver::solve(const FootPosition& foot) const
{
  const double cl = params_.coxa_length;
  const double fl = params_.femur_length;
  const double tl = params_.tibia_length;

  IKResult result;
  result.status = IKStatus::ERROR;

  // 1. Coxa angle: direction to foot in XY plane
  const double ca = std::atan2(foot.x, foot.y);

  // 2. Horizontal distance from body origin to foot, then subtract coxa
  const double r_total = std::sqrt(foot.x * foot.x + foot.y * foot.y);
  const double r = r_total - cl;

  // 3. Straight-line distance from coxa joint to foot
  const double d = std::sqrt(r * r + foot.z * foot.z);

  // 4. Reachability check
  const double max_reach = fl + tl;
  if (d > max_reach + 1e-9) {
    result.status = IKStatus::UNREACHABLE;
    return result;
  }

  // 5. Femur angle — elbow-down / knee-up solution
  //    phi1: angle of (r, fz) below/above horizontal
  //    phi2: angle at the coxa-tip vertex (femur side) from law of cosines
  const double phi1 = std::atan2(foot.z, r);
  double cos_phi2 = (fl * fl + d * d - tl * tl) / (2.0 * fl * d);
  cos_phi2 = std::max(-1.0, std::min(1.0, cos_phi2));
  const double phi2 = std::acos(cos_phi2);
  const double fa = phi1 - phi2;   // negative for leg pointing downward

  // 6. Tibia angle (interior angle at femur joint, converted to our convention)
  //    ta = π − acos(law-of-cosines)  → positive = knee bends up
  double cos_tibia = (fl * fl + tl * tl - d * d) / (2.0 * fl * tl);
  cos_tibia = std::max(-1.0, std::min(1.0, cos_tibia));
  const double ta = M_PI - std::acos(cos_tibia);

  result.status     = IKStatus::SUCCESS;
  result.joints.coxa  = ca;
  result.joints.femur = fa;
  result.joints.tibia = ta;
  return result;
}

}  // namespace hexapod_ik
