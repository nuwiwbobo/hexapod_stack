#ifndef HEXAPOD_GAIT__GAIT_ENGINE_HPP_
#define HEXAPOD_GAIT__GAIT_ENGINE_HPP_

#include <vector>
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace hexapod_gait
{

struct GaitParams
{
  int cycle_length = 50;
  double lift_height = 0.0375;
  double velocity_scaling = 0.15;
  double low_pass_alpha = 0.05;
};

class GaitEngine
{
public:
  explicit GaitEngine(const GaitParams & params);

  hexapod_msgs::msg::FeetPositions compute(const geometry_msgs::msg::Twist & cmd_vel);

private:
  void cyclePeriod(
    double base_x, double base_y, double base_theta,
    hexapod_msgs::msg::FeetPositions * feet);

  GaitParams params_;
  int cycle_period_ = 0;
  bool is_travelling_ = false;
  bool in_cycle_ = false;
  int extra_gait_cycle_ = 1;
  double smooth_x_ = 0.0;
  double smooth_y_ = 0.0;
  double smooth_theta_ = 0.0;

  // Tripod gait groups: 0=swing, 1=stance
  std::vector<int> cycle_leg_number_ = {1, 0, 1, 0, 1, 0};
};

}  // namespace hexapod_gait

#endif  // HEXAPOD_GAIT__GAIT_ENGINE_HPP_
