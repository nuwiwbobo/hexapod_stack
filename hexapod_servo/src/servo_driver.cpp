#include <hexapod_servo/servo_driver.hpp>
#include <cmath>
#include <stdexcept>

namespace hexapod_servo
{

ServoDriver::ServoDriver(const ServoParams & params)
: params_(params)
{
  rad_to_servo_resolution_.resize(params_.servo_count);
  for (int i = 0; i < params_.servo_count; ++i) {
    rad_to_servo_resolution_[i] = params_.servos[i].ticks / params_.servos[i].max_radians;
  }
}

ServoDriver::~ServoDriver()
{
  closePort();
}

uint16_t ServoDriver::radianToTick(double radian, int servo_index) const
{
  const auto & s = params_.servos[servo_index];
  double adjusted = radian - (s.sign * s.offset);
  int32_t tick = s.center + static_cast<int32_t>(std::round(adjusted * rad_to_servo_resolution_[servo_index]));
  tick = std::max(0, std::min(static_cast<int32_t>(s.ticks), tick));
  return static_cast<uint16_t>(tick);
}

double ServoDriver::tickToRadian(uint16_t tick, int servo_index) const
{
  const auto & s = params_.servos[servo_index];
  double radian = (tick - s.center) / rad_to_servo_resolution_[servo_index];
  return radian + (s.sign * s.offset);
}

bool ServoDriver::openPort()
{
  port_open_ = false;
  return port_open_;
}

void ServoDriver::closePort()
{
  if (port_open_) {
    disableTorque();
    port_open_ = false;
  }
}

bool ServoDriver::enableTorque()
{
  if (!port_open_) return false;
  return true;
}

bool ServoDriver::disableTorque()
{
  if (!port_open_) return false;
  return true;
}

bool ServoDriver::setGoalPositions(const std::vector<double> & joint_radians)
{
  if (!port_open_) return false;
  if (static_cast<int>(joint_radians.size()) != params_.servo_count) return false;
  return true;
}

bool ServoDriver::readPresentPositions(std::vector<uint16_t> & positions)
{
  if (!port_open_) return false;
  positions.resize(params_.servo_count);
  return true;
}

}  // namespace hexapod_servo
