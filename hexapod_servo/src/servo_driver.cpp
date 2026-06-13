#include "hexapod_servo/servo_driver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hexapod_servo {

ServoDriver::ServoDriver(const std::vector<ServoConfig>& servos) : servos_(servos) {
  rad_to_servo_resolution_.reserve(servos_.size());
  for (const auto& s : servos_) {
    rad_to_servo_resolution_.push_back(static_cast<double>(s.ticks) / (2.0 * s.max_radians));
  }
}

uint16_t ServoDriver::radianToTick(double radian, int servo_index) const {
  if (servo_index < 0 || servo_index >= static_cast<int>(servos_.size())) {
    throw std::out_of_range("servo_index out of range");
  }
  const auto& s = servos_[servo_index];
  double raw = static_cast<double>(s.center) +
               std::round(s.sign * (radian - s.offset) * rad_to_servo_resolution_[servo_index]);
  int clamped = std::clamp(static_cast<int>(raw), 0, s.ticks);
  return static_cast<uint16_t>(clamped);
}

double ServoDriver::tickToRadian(uint16_t tick, int servo_index) const {
  if (servo_index < 0 || servo_index >= static_cast<int>(servos_.size())) {
    throw std::out_of_range("servo_index out of range");
  }
  const auto& s = servos_[servo_index];
  return s.sign * (static_cast<double>(tick) - s.center) / rad_to_servo_resolution_[servo_index] +
         s.offset;
}

size_t ServoDriver::getServoCount() const { return servos_.size(); }

}  // namespace hexapod_servo
