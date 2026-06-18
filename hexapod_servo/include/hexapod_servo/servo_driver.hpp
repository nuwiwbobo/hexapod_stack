#ifndef HEXAPOD_SERVO__SERVO_DRIVER_HPP_
#define HEXAPOD_SERVO__SERVO_DRIVER_HPP_

// =============================================================================
// Task 1: Bare ServoDriver class (no ROS dependency)
// =============================================================================
// This class handles Dynamixel AX servo communication.
// It converts between radians and servo ticks, and manages servo configs.
//
// Implementation order:
//   1. ServoConfig struct (below)
//   2. ServoDriver class with radianToTick/tickToRadian
//   3. Unit tests in test/test_servo_driver.cpp
//
// Reference: ROS 1 servo driver at /home/wawabobo/ROS-package/hexapod_controller/include/servo_driver.h

#include <cstdint>
#include <string>
#include <vector>

namespace hexapod_servo {

// Servo configuration for a single Dynamixel AX servo
struct ServoConfig {
  int id;                   // Dynamixel servo ID (1-254)
  std::string type;         // "AX-12A" or "AX-18A"
  int ticks;                // Total tick range (1024 for AX-18A)
  int center;               // Center tick value (512 for AX-18A)
  double max_radians;       // Max rotation in radians (M_PI for 180°)
  double offset;            // Hardware offset in radians
  int sign;                 // +1 or -1 (flips direction for reversed mounting)
};

class ServoDriver {
public:
  explicit ServoDriver(const std::vector<ServoConfig>& servos);

  // =========================================================================
  // TODO: Implement these methods
  // =========================================================================

  // Convert radian angle to servo tick value
  // Formula: tick = center + round((radian - sign * offset) * (ticks / max_radians))
  // Clamp result to [0, ticks]
  uint16_t radianToTick(double radian, int servo_index) const;

  // Convert servo tick value back to radians
  // Formula: radian = (tick - center) / (ticks / max_radians) + sign * offset
  double tickToRadian(uint16_t tick, int servo_index) const;

  // Get number of configured servos
  size_t getServoCount() const;

  // =========================================================================
  // TODO: Implement these methods for hardware control (Task 2+)
  // =========================================================================

  // Open connection to U2D2 adapter
  // bool openPort(const std::string& port = "/dev/ttyUSB0", int baud = 1000000);
  // void closePort();

  // Enable/disable servo torque
  // bool enableTorque();
  // bool disableTorque();

  // Set goal position (radians) for one or all servos
  // bool setGoalPosition(int servo_id, double radians);
  // bool setGoalPositions(const std::vector<double>& radians);

  // Read current position from servo
  // double readPosition(int servo_id);
  // std::vector<double> readPositions();

private:
  std::vector<ServoConfig> servos_;
  std::vector<double> rad_to_servo_resolution_;  // Precomputed: ticks / max_radians
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_DRIVER_HPP_
