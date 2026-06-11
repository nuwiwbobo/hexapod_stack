#ifndef HEXAPOD_IK__IK_SOLVER_HPP_
#define HEXAPOD_IK__IK_SOLVER_HPP_

#include <cmath>
#include <vector>
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/msg/legs_joints.hpp>

namespace hexapod_ik
{

struct IkParams
{
  int number_of_legs = 6;
  double coxa_length = 0.044;
  double femur_length = 0.0545;
  double tibia_length = 0.1019;
  double tarsus_length = 0.0;
  double standing_body_height = 0.02;
  std::vector<double> init_coxa_angle;
  std::vector<double> coxa_to_center_x;
  std::vector<double> coxa_to_center_y;
  std::vector<double> init_foot_pos_x;
  std::vector<double> init_foot_pos_y;
  std::vector<double> init_foot_pos_z;
};

struct IkResult
{
  bool success = false;
  hexapod_msgs::msg::LegsJoints joints;
};

class IkSolver
{
public:
  explicit IkSolver(const IkParams & params);

  IkResult calculateIK(
    const hexapod_msgs::msg::FeetPositions & feet,
    const hexapod_msgs::msg::Pose & body);

private:
  struct Trig {
    double sine;
    double cosine;
  };

  static Trig getSinCos(double angle_rad);
  IkParams params_;
};

}  // namespace hexapod_ik

#endif  // HEXAPOD_IK__IK_SOLVER_HPP_
