#ifndef HEXAPOD_SERVO__SERVO_DRIVER_HPP_
#define HEXAPOD_SERVO__SERVO_DRIVER_HPP_

// =============================================================================
// ServoDriver with hardware control via dynamixel_sdk
// =============================================================================
// Wraps Dynamixel Protocol 1.0 communication for AX-series servos.
// Uses GroupSyncWrite/Read for efficient bus utilization.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dynamixel {
  class PortHandler;
  class PacketHandler;
  class GroupSyncWrite;
  class GroupSyncRead;
}

namespace hexapod_servo {

namespace ax_reg {
  constexpr int TORQUE_ENABLE = 24;
  constexpr int TORQUE_LIMIT  = 34;
  constexpr int MOVING_SPEED  = 32;
  constexpr int GOAL_POS      = 30;
  constexpr int PRESENT_POS   = 36;
  constexpr int MOVING        = 46;
}

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
  ~ServoDriver();

  uint16_t radianToTick(double radian, int servo_index) const;
  double   tickToRadian(uint16_t tick, int servo_index) const;
  size_t   getServoCount() const;

  bool openPort(const std::string& port = "/dev/ttyUSB0", int baud = 1000000);
  void closePort();
  bool isOpen() const { return port_open_; }

  bool enableTorque(int moving_speed = 100, int torque_limit = 300);
  bool disableTorque();

  bool setGoalPosition(int servo_index, double radians);
  bool setGoalPositions(const std::vector<double>& radians);

  double readPosition(int servo_index);
  std::vector<double> readPositions();

private:
  std::vector<ServoConfig> servos_;
  std::vector<double>      resolution_;
  bool                     port_open_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  dynamixel::GroupSyncWrite* sync_write_{nullptr};
  dynamixel::GroupSyncRead*  sync_read_{nullptr};
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_DRIVER_HPP_
