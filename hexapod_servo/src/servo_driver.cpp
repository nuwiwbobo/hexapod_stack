// =============================================================================
// servo_driver.cpp
// =============================================================================
#include <hexapod_servo/servo_driver.hpp> // Include the header file for the ServoDriver class

#include <algorithm> // For std::max and std::min
#include <cmath>     // For std::round
#include <cstring>   // For std::memcpy
#include <limits>    // For std::numeric_limits
#include <stdexcept>  // For std::invalid_argument

#include <dynamixel_sdk/dynamixel_sdk.h> // Include the Dynamixel SDK for communication with the servos

namespace hexapod_servo {

ServoDriver::ServoDriver(const std::vector<ServoConfig>& servos)
: servos_(servos) // Precompute resolution (radians per tick) for each servo based on its configuration
{
  if (servos_.empty()) {
    throw std::invalid_argument("ServoDriver: servo list must not be empty");
  }
  resolution_.reserve(servos_.size()); // Preallocate resolution vector
  for (const auto& s : servos_) {
    if (s.max_radians <= 0.0) {        // Validate max_radians to prevent division by zero
      throw std::invalid_argument(
        "ServoDriver: max_radians must be positive (id=" + std::to_string(s.id) + ")");
    }
    resolution_.push_back(static_cast<double>(s.ticks) / s.max_radians); // Calculate resolution as ticks per radian
  }                                                                      // Initialize packet handler for Dynamixel Protocol 1.0 (used by AX-series servos)
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);     // Note: Protocol 2.0 is not compatible with AX-series servos, so we use 1.0 here
}

ServoDriver::~ServoDriver() { closePort(); } // Ensure the serial port is closed and resources are cleaned up when the ServoDriver is destroyed

// ---------------------------------------------------------------------------
// Unit conversion between radians and servo ticks
// ---------------------------------------------------------------------------
uint16_t ServoDriver::radianToTick(double radian, int servo_index) const  // Convert a desired position in radians to the corresponding servo tick value based on the servo's configuration and resolution
{
  const ServoConfig& s   = servos_.at(servo_index);                       // Get the servo configuration for the specified index
  const double       res = resolution_.at(servo_index);                   // Get the precomputed resolution (ticks per radian) for this servo
  double raw = static_cast<double>(s.center)                              // Calculate the raw tick value by applying the resolution and offset, and considering the sign for direction control
             + static_cast<double>(s.sign) * (radian - s.offset) * res;   // Clamp the raw tick value to the valid range of [0, s.ticks] to prevent sending out-of-range values to the servo
  int clamped = static_cast<int>(std::round(raw));                        // Round the raw tick value to the nearest integer and clamp it to the valid range of [0, s.ticks]
  clamped = std::max(0, std::min(clamped, s.ticks));                      // Return the clamped tick value as an unsigned 16-bit integer
  return static_cast<uint16_t>(clamped);
}
// Example: For an AX-18A servo with 1024 ticks for a full rotation (2.61799 radians), a center at 512 ticks, and no offset, the resolution would be approximately 390.625 ticks per radian. 
// If we want to set the servo to 1 radian, the raw tick value would be 512 + 1 * 390.625 = 902.625, which would be rounded to 903 ticks.

double ServoDriver::tickToRadian(uint16_t tick, int servo_index) const    // Convert a servo tick value back to radians using the servo's configuration and resolution
{
  const ServoConfig& s   = servos_.at(servo_index);
  const double       res = resolution_.at(servo_index);
  return (static_cast<double>(tick) - static_cast<double>(s.center))
       / (static_cast<double>(s.sign) * res)
       + s.offset;
}

size_t ServoDriver::getServoCount() const { return servos_.size(); }      // Return the number of servos managed by this driver, which is determined by the size of the servos_ vector

// ---------------------------------------------------------------------------
// Hardware — port
// ---------------------------------------------------------------------------
bool ServoDriver::openPort(const std::string& port, int baud)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port.c_str());
  if (!port_handler_->openPort())      return false;
  if (!port_handler_->setBaudRate(baud)) {
    port_handler_->closePort(); return false;
  }
  port_open_ = true;

  // Create sync write/read objects (2 bytes per servo: GOAL_POS / PRESENT_POS)
  sync_write_ = new dynamixel::GroupSyncWrite(
    port_handler_, packet_handler_, ax_reg::GOAL_POS, 2);
  sync_read_ = new dynamixel::GroupSyncRead(
    port_handler_, packet_handler_, ax_reg::PRESENT_POS, 2);

  // Register all servos with sync_read so we can batch-read them
  for (const auto& s : servos_) {
    sync_read_->addParam(s.id);
  }

  return true;
}

void ServoDriver::closePort()
{
  if (port_open_ && port_handler_) {
    disableTorque();
    delete sync_write_; sync_write_ = nullptr;
    delete sync_read_;  sync_read_  = nullptr;
    port_handler_->closePort();
    port_open_ = false;
  }
}

// ---------------------------------------------------------------------------
// Hardware — torque
// ---------------------------------------------------------------------------
bool ServoDriver::enableTorque(int moving_speed, int torque_limit)
{
  if (!port_open_) return false;
  bool ok = true;
  uint8_t err = 0;
  for (const auto& s : servos_) {
    packet_handler_->write2ByteTxRx(
      port_handler_, s.id, ax_reg::MOVING_SPEED,
      static_cast<uint16_t>(moving_speed), &err);
    packet_handler_->write2ByteTxRx(
      port_handler_, s.id, ax_reg::TORQUE_LIMIT,
      static_cast<uint16_t>(torque_limit), &err);
    int rc = packet_handler_->write1ByteTxRx(
      port_handler_, s.id, ax_reg::TORQUE_ENABLE, 1, &err);
    if (rc != COMM_SUCCESS || err) ok = false;
  }
  return ok;
}

bool ServoDriver::disableTorque()
{
  if (!port_open_) return false;
  bool ok = true;
  uint8_t err = 0;
  for (const auto& s : servos_) {
    int rc = packet_handler_->write1ByteTxRx(
      port_handler_, s.id, ax_reg::TORQUE_ENABLE, 0, &err);
    if (rc != COMM_SUCCESS || err) ok = false;
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Hardware — goal position
// ---------------------------------------------------------------------------
bool ServoDriver::setGoalPosition(int servo_index, double radians)
{
  if (!port_open_) return false;
  const ServoConfig& s = servos_.at(servo_index);
  uint16_t tick = radianToTick(radians, servo_index);
  uint8_t err = 0;
  int rc = packet_handler_->write2ByteTxRx(
    port_handler_, s.id, ax_reg::GOAL_POS, tick, &err);
  return (rc == COMM_SUCCESS && !err);
}

bool ServoDriver::setGoalPositions(const std::vector<double>& radians)
{
  if (radians.size() != servos_.size()) return false;
  if (!port_open_ || !sync_write_) return false;

  // Clear previous parameters
  sync_write_->clearParam();

  // Pack all goal positions into the sync write packet
  for (size_t i = 0; i < servos_.size(); ++i) {
    uint16_t tick = radianToTick(radians[i], static_cast<int>(i));
    uint8_t data[2] = {
      static_cast<uint8_t>(tick & 0xFF),
      static_cast<uint8_t>((tick >> 8) & 0xFF)
    };
    sync_write_->addParam(servos_[i].id, data);
  }

  // Send one packet to all servos
  int rc = sync_write_->txPacket();
  sync_write_->clearParam();
  return (rc == COMM_SUCCESS);
}

// ---------------------------------------------------------------------------
// Hardware — read position
// ---------------------------------------------------------------------------
double ServoDriver::readPosition(int servo_index)
{
  if (!port_open_) return std::numeric_limits<double>::quiet_NaN();
  const ServoConfig& s = servos_.at(servo_index);
  uint16_t tick = 0;
  uint8_t  err  = 0;
  int rc = packet_handler_->read2ByteTxRx(
    port_handler_, s.id, ax_reg::PRESENT_POS, &tick, &err);
  if (rc != COMM_SUCCESS || err) return std::numeric_limits<double>::quiet_NaN();
  return tickToRadian(tick, servo_index);
}

std::vector<double> ServoDriver::readPositions()
{
  std::vector<double> positions(servos_.size(),
    std::numeric_limits<double>::quiet_NaN());
  if (!port_open_) return positions;

  // Protocol 1.0 doesn't support sync read — use individual reads
  for (size_t i = 0; i < servos_.size(); ++i) {
    positions[i] = readPosition(static_cast<int>(i));
  }
  return positions;
}

}  // namespace hexapod_servo
