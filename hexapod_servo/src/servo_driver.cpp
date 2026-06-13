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
  if (port_open_) {
    closePort();
  }

  port_handler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(params_.protocol_version);

  if (!port_handler_->openPort()) {
    fprintf(stderr, "[ServoDriver] Failed to open port /dev/ttyUSB0\n");
    delete port_handler_;
    port_handler_ = nullptr;
    delete packet_handler_;
    packet_handler_ = nullptr;
    return false;
  }

  if (!port_handler_->setBaudRate(params_.baud_rate)) {
    fprintf(stderr, "[ServoDriver] Failed to set baud rate %d\n", params_.baud_rate);
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
    delete packet_handler_;
    packet_handler_ = nullptr;
    return false;
  }

  port_open_ = true;
  fprintf(stdout, "[ServoDriver] Port /dev/ttyUSB0 opened at %d baud\n", params_.baud_rate);
  return true;
}

void ServoDriver::closePort()
{
  if (port_open_) {
    disableTorque();
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
    delete packet_handler_;
    packet_handler_ = nullptr;
    port_open_ = false;
  }
}

bool ServoDriver::enableTorque()
{
  if (!port_open_) return false;

  for (int i = 0; i < params_.servo_count; ++i) {
    uint8_t dxl_error = 0;
    int result = packet_handler_->write1ByteTxRx(
      port_handler_,
      params_.servos[i].id,
      params_.torque_enable_reg,
      1,  // enable
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      fprintf(stderr, "[ServoDriver] Failed to enable torque on servo %d (error: %d)\n",
              params_.servos[i].id, dxl_error);
      return false;
    }
  }

  fprintf(stdout, "[ServoDriver] Torque enabled on all %d servos\n", params_.servo_count);
  return true;
}

bool ServoDriver::disableTorque()
{
  if (!port_open_) return false;

  for (int i = 0; i < params_.servo_count; ++i) {
    uint8_t dxl_error = 0;
    int result = packet_handler_->write1ByteTxRx(
      port_handler_,
      params_.servos[i].id,
      params_.torque_enable_reg,
      0,  // disable
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      fprintf(stderr, "[ServoDriver] Failed to disable torque on servo %d (error: %d)\n",
              params_.servos[i].id, dxl_error);
      return false;
    }
  }

  fprintf(stdout, "[ServoDriver] Torque disabled on all %d servos\n", params_.servo_count);
  return true;
}

bool ServoDriver::setGoalPositions(const std::vector<double> & joint_radians)
{
  if (!port_open_) return false;
  if (static_cast<int>(joint_radians.size()) != params_.servo_count) return false;

  for (int i = 0; i < params_.servo_count; ++i) {
    uint16_t tick = radianToTick(joint_radians[i], i);
    uint8_t dxl_error = 0;
    int result = packet_handler_->write2ByteTxRx(
      port_handler_,
      params_.servos[i].id,
      params_.goal_position_reg,
      tick,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      fprintf(stderr, "[ServoDriver] Failed to set goal position on servo %d (error: %d)\n",
              params_.servos[i].id, dxl_error);
      return false;
    }
  }

  return true;
}

bool ServoDriver::readPresentPositions(std::vector<uint16_t> & positions)
{
  if (!port_open_) return false;

  positions.resize(params_.servo_count);

  for (int i = 0; i < params_.servo_count; ++i) {
    uint16_t position = 0;
    uint8_t dxl_error = 0;
    int result = packet_handler_->read2ByteTxRx(
      port_handler_,
      params_.servos[i].id,
      params_.present_position_reg,
      &position,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      fprintf(stderr, "[ServoDriver] Failed to read position from servo %d (error: %d)\n",
              params_.servos[i].id, dxl_error);
      return false;
    }

    positions[i] = position;
  }

  return true;
}

}  // namespace hexapod_servo
