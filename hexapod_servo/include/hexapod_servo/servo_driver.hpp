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
  class PortHandler; //Serial port communication handler (e.g., /dev/ttyUSB0)
  class PacketHandler; //Dynamixel packet protocol handler (e.g., Protocol 1.0)
  class GroupSyncWrite; //Batch write for multiple servos (e.g., goal position)
  class GroupSyncRead; //Batch read for multiple servos (e.g., present position)
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
  int id;             // Dynamixel ID (1-254)
  std::string type;   // Servo type (e.g., "AX-18A")
  int ticks;          // Total ticks for full rotation (e.g., 1024 for AX-18A)
  int center;         // Center tick value (e.g., 512 for AX-18A)
  double max_radians; // Maximum rotation in radians (e.g., 2.61799 for AX-18A)
  double offset;      // Offset for position calibration
  int sign;           // Sign for direction control
};

class ServoDriver {
public:
  // Constructor takes a vector of ServoConfig to initialize the driver
  explicit ServoDriver(const std::vector<ServoConfig>& servos);
  ~ServoDriver();

  // Convert between radians and servo ticks based on servo configuration
  uint16_t radianToTick(double radian, int servo_index) const;
  double   tickToRadian(uint16_t tick, int servo_index) const;
  size_t   getServoCount() const;

  // Open and close the serial port for communication with the servos
  bool openPort(const std::string& port = "/dev/ttyUSB0", int baud = 1000000);
  void closePort();
  bool isOpen() const { return port_open_; }

  // Enable or disable torque on all servos with optional speed and torque limit
  bool enableTorque(int moving_speed = 100, int torque_limit = 300);
  bool disableTorque();

  // Set goal position for a single servo or multiple servos in radians
  bool setGoalPosition(int servo_index, double radians);
  bool setGoalPositions(const std::vector<double>& radians);

  // Read current position of a single servo or all servos in radians
  double readPosition(int servo_index);
  std::vector<double> readPositions();

private:
  std::vector<ServoConfig> servos_;           // Configuration for each servo
  std::vector<double>      resolution_;       // radians per tick for each servo
  bool                     port_open_{false}; // Flag indicating if the serial port is open

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  dynamixel::GroupSyncWrite* sync_write_{nullptr};
  dynamixel::GroupSyncRead*  sync_read_{nullptr};
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_DRIVER_HPP_
