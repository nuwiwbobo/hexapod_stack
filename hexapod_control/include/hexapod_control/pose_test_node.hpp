#ifndef HEXAPOD_CONTROL__POSE_TEST_NODE_HPP_
#define HEXAPOD_CONTROL__POSE_TEST_NODE_HPP_

#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hexapod_ik/ik_solver.hpp>

namespace hexapod_control {

struct LegMount {
  double mount_x;
  double mount_y;
  double coxa_angle;
};

enum class PoseMode { STANCE, SWEEP_COXA, SWEEP_FEMUR, SWEEP_TIBIA };

class PoseTestNode : public rclcpp::Node {
public:
  PoseTestNode();
  ~PoseTestNode();
  void run();

private:
  void controlLoop();
  void handleKey(char key);
  void printStanceInfo();
  hexapod_ik::FootPosition bodyToLeg(
    const hexapod_ik::FootPosition& body_foot, int leg_idx) const;
  hexapod_ik::FootPosition legToBody(
    const hexapod_ik::FootPosition& leg_foot, int leg_idx) const;

  hexapod_ik::IkSolver ik_solver_;
  std::array<LegMount, 6> mounts_;
  std::array<hexapod_ik::LegJoints, 6> default_joints_;
  std::array<hexapod_ik::FootPosition, 6> body_stance_;
  std::vector<std::string> all_joint_names_;

  PoseMode mode_{PoseMode::STANCE};
  double sweep_amplitude_{0.4};
  double sweep_period_{6.0};
  double publish_rate_{20.0};
  double sweep_phase_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool running_{true};
  int publish_count_{0};
};

}  // namespace hexapod_control

#endif  // HEXAPOD_CONTROL__POSE_TEST_NODE_HPP_
