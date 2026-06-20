// =============================================================================
// servo_driver.cpp
// =============================================================================
#include <hexapod_servo/servo_driver.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace hexapod_servo {

ServoDriver::ServoDriver(const std::vector<ServoConfig>& servos)
: servos_(servos)
{
  if (servos_.empty()) {
    throw std::invalid_argument("ServoDriver: servo list must not be empty");
  }
  resolution_.reserve(servos_.size());
  for (const auto& s : servos_) {
    if (s.max_radians <= 0.0) {
      throw std::invalid_argument(
        "ServoDriver: max_radians must be positive (id=" + std::to_string(s.id) + ")");
    }
    resolution_.push_back(static_cast<double>(s.ticks) / s.max_radians);
  }
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
}

ServoDriver::~ServoDriver() { closePort(); }

// ---------------------------------------------------------------------------
// Unit conversion
// ---------------------------------------------------------------------------
uint16_t ServoDriver::radianToTick(double radian, int servo_index) const
{
  const ServoConfig& s   = servos_.at(servo_index);
  const double       res = resolution_.at(servo_index);
  double raw = static_cast<double>(s.center)
             + static_cast<double>(s.sign) * (radian - s.offset) * res;
  int clamped = static_cast<int>(std::round(raw));
  clamped = std::max(0, std::min(clamped, s.ticks));
  return static_cast<uint16_t>(clamped);
}

double ServoDriver::tickToRadian(uint16_t tick, int servo_index) const
{
  const ServoConfig& s   = servos_.at(servo_index);
  const double       res = resolution_.at(servo_index);
  return (static_cast<double>(tick) - static_cast<double>(s.center))
       / (static_cast<double>(s.sign) * res)
       + s.offset;
}

size_t ServoDriver::getServoCount() const { return servos_.size(); }

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
