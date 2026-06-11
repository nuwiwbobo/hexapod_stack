#ifndef HEXAPOD_SERVO__SERVO_DRIVER_HPP_
#define HEXAPOD_SERVO__SERVO_DRIVER_HPP_

#include <vector>
#include <string>
#include <cstdint>

namespace hexapod_servo
{

struct ServoConfig
{
  int id = 0;
  int ticks = 1024;
  int center = 512;
  double max_radians = 5.236;
  int sign = 1;
  double offset = 0.0;
};

struct ServoParams
{
  int servo_count = 18;
  int baud_rate = 1000000;
  double protocol_version = 1.0;
  int torque_enable_reg = 24;
  int present_position_reg = 36;
  int goal_position_reg = 30;
  std::vector<ServoConfig> servos;
  std::vector<std::string> joint_names;
};

class ServoDriver
{
public:
  explicit ServoDriver(const ServoParams & params);
  ~ServoDriver();

  ServoDriver(const ServoDriver &) = delete;
  ServoDriver & operator=(const ServoDriver &) = delete;

  uint16_t radianToTick(double radian, int servo_index) const;
  double tickToRadian(uint16_t tick, int servo_index) const;

  bool openPort();
  void closePort();
  bool enableTorque();
  bool disableTorque();
  bool setGoalPositions(const std::vector<double> & joint_radians);
  bool readPresentPositions(std::vector<uint16_t> & positions);

private:
  ServoParams params_;
  void * port_handler_ = nullptr;
  void * packet_handler_ = nullptr;
  bool port_open_ = false;
  std::vector<double> rad_to_servo_resolution_;
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_DRIVER_HPP_
