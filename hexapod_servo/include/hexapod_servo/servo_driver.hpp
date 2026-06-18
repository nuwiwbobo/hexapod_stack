#ifndef HEXAPOD_SERVO__SERVO_DRIVER_HPP_
#define HEXAPOD_SERVO__SERVO_DRIVER_HPP_

// =============================================================================
// Task 2: ServoDriver with hardware control via dynamixel_sdk
// =============================================================================
// Wraps Dynamixel Protocol 1.0 communication for AX-series servos.
// Unit conversion (radianToTick / tickToRadian) has no SDK dependency.
// Hardware methods require openPort() first.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Forward-declare SDK types to keep this header ROS-free
namespace dynamixel {
  class PortHandler;
  class PacketHandler;
}

namespace hexapod_servo {

// AX-series register addresses (Protocol 1.0)
namespace ax_reg {
  constexpr int TORQUE_ENABLE = 24;
  constexpr int TORQUE_LIMIT  = 34;
  constexpr int MOVING_SPEED  = 32;
  constexpr int GOAL_POS      = 30;
  constexpr int PRESENT_POS   = 36;
  constexpr int MOVING        = 46;
}

struct ServoConfig {
  int id;                   // Dynamixel servo ID (1-254)
  std::string type;         // "AX-12A" or "AX-18A"
  int ticks;                // Total tick range (1024 for AX series)
  int center;               // Center tick value (512)
  double max_radians;       // Logical range in radians (use 2*pi convention)
  double offset;            // Hardware zero offset in radians
  int sign;                 // +1 or -1 (flips direction for reversed mounting)
};

class ServoDriver {
public:
  explicit ServoDriver(const std::vector<ServoConfig>& servos);
  ~ServoDriver();

  // ---------------------------------------------------------------------------
  // Unit conversion (no hardware dependency)
  // ---------------------------------------------------------------------------
  uint16_t radianToTick(double radian, int servo_index) const;
  double   tickToRadian(uint16_t tick, int servo_index) const;
  size_t   getServoCount() const;

  // ---------------------------------------------------------------------------
  // Hardware control (call openPort() first)
  // ---------------------------------------------------------------------------

  // Open U2D2 connection. port = "/dev/ttyUSB0", baud = 1000000
  bool openPort(const std::string& port = "/dev/ttyUSB0", int baud = 1000000);
  void closePort();
  bool isOpen() const { return port_open_; }

  // Torque — applies to ALL configured servos
  bool enableTorque(int moving_speed = 100, int torque_limit = 300);
  bool disableTorque();

  // Set goal position for one servo (by servo_index, not ID)
  bool setGoalPosition(int servo_index, double radians);

  // Set goal positions for all servos in one pass
  bool setGoalPositions(const std::vector<double>& radians);

  // Read present position of one servo (returns NaN on error)
  double readPosition(int servo_index);

  // Read all servo positions (NaN entries on error)
  std::vector<double> readPositions();

private:
  std::vector<ServoConfig> servos_;
  std::vector<double>      resolution_;   // precomputed ticks / max_radians
  bool                     port_open_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_DRIVER_HPP_
