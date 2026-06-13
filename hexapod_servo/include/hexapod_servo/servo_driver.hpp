#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hexapod_servo {

struct ServoConfig {
  int id;
  std::string type;
  int ticks;
  int center;
  double max_radians;
  double offset;
  int sign;
};

class ServoDriver {
public:
  explicit ServoDriver(const std::vector<ServoConfig>& servos);
  uint16_t radianToTick(double radian, int servo_index) const;
  double tickToRadian(uint16_t tick, int servo_index) const;
  size_t getServoCount() const;

private:
  std::vector<ServoConfig> servos_;
  std::vector<double> rad_to_servo_resolution_;
};

}  // namespace hexapod_servo
