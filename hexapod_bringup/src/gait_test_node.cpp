// =============================================================================
// gait_test_node.cpp — Single-leg walking gait test
// =============================================================================
// Generates a rectangular foot trajectory for one leg:
//   Phase 0: Lift foot up
//   Phase 1: Swing forward
//   Phase 2: Lower foot
//   Phase 3: Swing back
//
// Publishes to /joint_targets.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include <hexapod_ik/ik_solver.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class GaitTestNode : public rclcpp::Node
{
public:
  GaitTestNode()
  : Node("gait_test_node"), phase_(0), phase_step_(0)
  {
    declare_parameter("config_file", "");
    declare_parameter("leg_key", "LR");
    declare_parameter("lift_height", 0.03);
    declare_parameter("step_length", 0.03);
    declare_parameter("cycle_hz", 1.0);

    const std::string config_file = get_parameter("config_file").as_string();
    leg_key_ = get_parameter("leg_key").as_string();
    lift_height_ = get_parameter("lift_height").as_double();
    step_length_ = get_parameter("step_length").as_double();
    cycle_hz_ = get_parameter("cycle_hz").as_double();

    if (config_file.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'config_file' not set");
      return;
    }

    YAML::Node config;
    try {
      config = YAML::LoadFile(config_file);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse YAML: %s", e.what());
      return;
    }

    // --- IK solver ---
    auto ik_node = config["hexapod_ik"];
    hexapod_ik::LegParams leg_params;
    leg_params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
    leg_params.femur_length = ik_node["femur_length"].as<double>(0.0545);
    leg_params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);
    ik_solver_ = std::make_unique<hexapod_ik::IkSolver>(leg_params);

    // --- Joint names ---
    auto joint_names_node = ik_node["joint_names"];
    if (joint_names_node) {
      joint_names_ = joint_names_node.as<std::vector<std::string>>();
    } else {
      joint_names_ = {
        "coxa_joint_" + leg_key_,
        "femur_joint_" + leg_key_,
        "tibia_joint_" + leg_key_};
    }

    // --- Default stance ---
    default_foot_.x = step_length_;
    default_foot_.y = 0.0;
    default_foot_.z = -0.10;

    // --- Publisher + timer ---
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

    double period = 1.0 / (cycle_hz_ * 4.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&GaitTestNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "Gait test ready — leg=%s lift=%.3f step=%.3f hz=%.2f",
      leg_key_.c_str(), lift_height_, step_length_, cycle_hz_);
  }

private:
  void controlLoop()
  {
    hexapod_ik::FootPosition foot = default_foot_;
    const double t = static_cast<double>(phase_step_) / steps_per_phase_;

    switch (phase_) {
      case 0:  // Lift
        foot.z = default_foot_.z + lift_height_ * t;
        break;
      case 1:  // Swing forward
        foot.x = default_foot_.x + step_length_ * t;
        foot.z = default_foot_.z + lift_height_;
        break;
      case 2:  // Lower
        foot.z = default_foot_.z + lift_height_ * (1.0 - t);
        foot.x = default_foot_.x + step_length_;
        break;
      case 3:  // Swing back
        foot.x = default_foot_.x + step_length_ * (1.0 - t);
        break;
    }

    auto result = ik_solver_->solve(foot);

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = now();

    if (result.status == hexapod_ik::IKStatus::SUCCESS) {
      joint_msg.name = joint_names_;
      joint_msg.position = {result.joints.coxa, result.joints.femur, result.joints.tibia};
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "IK failed at phase %d step %d — holding", phase_, phase_step_);
      auto def = ik_solver_->solve(default_foot_);
      if (def.status == hexapod_ik::IKStatus::SUCCESS) {
        joint_msg.name = joint_names_;
        joint_msg.position = {def.joints.coxa, def.joints.femur, def.joints.tibia};
      }
    }

    joint_pub_->publish(joint_msg);

    phase_step_++;
    if (phase_step_ >= steps_per_phase_) {
      phase_step_ = 0;
      phase_ = (phase_ + 1) % 4;
    }
  }

  std::unique_ptr<hexapod_ik::IkSolver> ik_solver_;

  std::string leg_key_;
  std::vector<std::string> joint_names_;
  hexapod_ik::FootPosition default_foot_;

  double lift_height_;
  double step_length_;
  double cycle_hz_;

  int phase_;
  int phase_step_;
  static constexpr int steps_per_phase_ = 25;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
