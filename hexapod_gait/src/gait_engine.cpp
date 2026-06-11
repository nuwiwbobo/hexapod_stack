#include <hexapod_gait/gait_engine.hpp>
#include <cmath>
#include <algorithm>

namespace hexapod_gait
{

constexpr double PI = 3.14159265358979323846;

GaitEngine::GaitEngine(const GaitParams & params)
: params_(params)
{
}

hexapod_msgs::msg::FeetPositions GaitEngine::compute(const geometry_msgs::msg::Twist & cmd_vel)
{
  double vx = cmd_vel.linear.x * params_.velocity_scaling;
  double vy = cmd_vel.linear.y * params_.velocity_scaling;
  double vth = cmd_vel.angular.z * params_.velocity_scaling;

  double base_x = vx / PI * params_.cycle_length;
  double base_y = vy / PI * params_.cycle_length;
  double base_theta = vth / PI * params_.cycle_length;

  smooth_x_ = base_x * params_.low_pass_alpha + smooth_x_ * (1.0 - params_.low_pass_alpha);
  smooth_y_ = base_y * params_.low_pass_alpha + smooth_y_ * (1.0 - params_.low_pass_alpha);
  smooth_theta_ = base_theta * params_.low_pass_alpha + smooth_theta_ * (1.0 - params_.low_pass_alpha);

  if (std::abs(smooth_x_) > 0.001 || std::abs(smooth_y_) > 0.001 ||
      std::abs(smooth_theta_) > 0.00436332313)
  {
    is_travelling_ = true;
  } else {
    is_travelling_ = false;
  }

  hexapod_msgs::msg::FeetPositions feet;
  for (int i = 0; i < 6; ++i) {
    feet.foot[i].position.x = 0.0;
    feet.foot[i].position.y = 0.0;
    feet.foot[i].position.z = 0.0;
    feet.foot[i].orientation.yaw = 0.0;
  }

  if (is_travelling_ || in_cycle_) {
    cyclePeriod(smooth_x_, smooth_y_, smooth_theta_, &feet);
    cycle_period_++;
  } else {
    cycle_period_ = 0;
  }

  if (cycle_period_ == params_.cycle_length) {
    cycle_period_ = 0;
    for (auto & leg : cycle_leg_number_) {
      leg = (leg == 0) ? 1 : 0;
    }
  }

  return feet;
}

void GaitEngine::cyclePeriod(
  double base_x, double base_y, double base_theta,
  hexapod_msgs::msg::FeetPositions * feet)
{
  double period_height = std::sin(cycle_period_ * PI / params_.cycle_length);

  for (int leg_index = 0; leg_index < 6; ++leg_index)
  {
    if (cycle_leg_number_[leg_index] == 0 && is_travelling_) {
      double period_distance = std::cos(cycle_period_ * PI / params_.cycle_length);
      feet->foot[leg_index].position.x = base_x * period_distance;
      feet->foot[leg_index].position.y = base_y * period_distance;
      feet->foot[leg_index].position.z = params_.lift_height * period_height;
      feet->foot[leg_index].orientation.yaw = base_theta * period_distance;
    }
    if (cycle_leg_number_[leg_index] == 1) {
      double period_distance = std::cos(cycle_period_ * PI / params_.cycle_length);
      feet->foot[leg_index].position.x = -base_x * period_distance;
      feet->foot[leg_index].position.y = -base_y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -base_theta * period_distance;
    }
  }
}

}  // namespace hexapod_gait
