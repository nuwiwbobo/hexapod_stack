#include <hexapod_ik/ik_solver.hpp>
#include <algorithm>

namespace hexapod_ik
{

IkSolver::IkSolver(const IkParams & params)
: params_(params)
{
}

IkSolver::Trig IkSolver::getSinCos(double angle_rad)
{
  return {std::sin(angle_rad), std::cos(angle_rad)};
}

IkResult IkSolver::calculateIK(
  const hexapod_msgs::msg::FeetPositions & feet,
  const hexapod_msgs::msg::Pose & body)
{
  IkResult result;
  result.success = true;

  constexpr double PI = 3.14159265358979323846;

  for (int leg_index = 0; leg_index < params_.number_of_legs; ++leg_index)
  {
    double sign = (leg_index <= 2) ? -1.0 : 1.0;

    Trig A = getSinCos(body.orientation.yaw + feet.foot[leg_index].orientation.yaw);
    Trig B = getSinCos(body.orientation.pitch);
    Trig G = getSinCos(body.orientation.roll);

    double cpr_x = feet.foot[leg_index].position.x + body.position.x
                  - params_.init_foot_pos_x[leg_index] - params_.coxa_to_center_x[leg_index];
    double cpr_y = feet.foot[leg_index].position.y + sign * (body.position.y
                  + params_.init_foot_pos_y[leg_index] + params_.coxa_to_center_y[leg_index]);
    double cpr_z = feet.foot[leg_index].position.z + body.position.z
                  + params_.tarsus_length - params_.init_foot_pos_z[leg_index];

    double body_pos_x = cpr_x - (
      cpr_x * A.cosine * B.cosine +
      cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine +
      cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine
    );
    double body_pos_y = cpr_y - (
      cpr_x * B.cosine * A.sine +
      cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine +
      cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine
    );
    double body_pos_z = cpr_z - (
      -cpr_x * B.sine +
      cpr_y * B.cosine * G.sine +
      cpr_z * B.cosine * G.cosine
    );

    double feet_pos_x = -params_.init_foot_pos_x[leg_index] + body.position.x
                      - body_pos_x + feet.foot[leg_index].position.x;
    double feet_pos_y = params_.init_foot_pos_y[leg_index] + sign * (body.position.y
                      - body_pos_y + feet.foot[leg_index].position.y);
    double feet_pos_z = params_.init_foot_pos_z[leg_index] - params_.tarsus_length
                      + body.position.z - body_pos_z - feet.foot[leg_index].position.z;

    double femur_to_tarsus = std::sqrt(
      feet_pos_x * feet_pos_x + feet_pos_y * feet_pos_y
    ) - params_.coxa_length;

    double full_dist = std::sqrt(
      femur_to_tarsus * femur_to_tarsus + feet_pos_z * feet_pos_z
    );

    if (full_dist > (params_.femur_length + params_.tibia_length))
    {
      result.success = false;
      return result;
    }

    double side_a = params_.femur_length;
    double side_a_sqr = side_a * side_a;
    double side_b = params_.tibia_length;
    double side_b_sqr = side_b * side_b;
    double side_c = full_dist;
    double side_c_sqr = side_c * side_c;

    double angle_b = std::acos(
      std::clamp((side_a_sqr - side_b_sqr + side_c_sqr) / (2.0 * side_a * side_c), -1.0, 1.0)
    );
    double angle_c = std::acos(
      std::clamp((side_a_sqr + side_b_sqr - side_c_sqr) / (2.0 * side_a * side_b), -1.0, 1.0)
    );

    double theta = std::atan2(femur_to_tarsus, feet_pos_z);

    result.joints.leg[leg_index].coxa =
      std::atan2(feet_pos_x, feet_pos_y) + params_.init_coxa_angle[leg_index];
    result.joints.leg[leg_index].femur = (PI / 2.0) - (theta + angle_b);
    result.joints.leg[leg_index].tibia = (PI / 2.0) - angle_c;
  }

  return result;
}

}  // namespace hexapod_ik
