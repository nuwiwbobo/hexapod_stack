// =============================================================================
// control_node.cpp — Orchestrator: cmd_vel → gait → IK → /joint_targets
// =============================================================================
// Loads config from YAML via yaml-cpp.
// No ServoDriver — hardware handled by servo_node.

#include <hexapod_control/control_node.hpp>

#include <chrono>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace hexapod_control {

ControlNode::ControlNode()
: Node("control_node"),
  gait_generator_({48, 0.03, 0.05}),
  ik_solvers_({hexapod_ik::IkSolver({0.044, 0.0545, 0.1019}),
               hexapod_ik::IkSolver({0.044, 0.0545, 0.1019}),
               hexapod_ik::IkSolver({0.044, 0.0545, 0.1019}),
               hexapod_ik::IkSolver({0.044, 0.0545, 0.1019}),
               hexapod_ik::IkSolver({0.044, 0.0545, 0.1019}),
               hexapod_ik::IkSolver({0.044, 0.0545, 0.1019})})
{
  // --- Load YAML config ---
  declare_parameter("config_file", "");
  const std::string config_file = get_parameter("config_file").as_string();

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

  // --- Load IK params ---
  auto ik_node = config["hexapod_ik"];
  if (ik_node) {
    hexapod_ik::LegParams params;
    params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
    params.femur_length = ik_node["femur_length"].as<double>(0.0545);
    params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);

    for (auto& solver : ik_solvers_) {
      solver = hexapod_ik::IkSolver(params);
    }
    RCLCPP_INFO(get_logger(), "IK params: coxa=%.3f femur=%.3f tibia=%.3f",
      params.coxa_length, params.femur_length, params.tibia_length);
  }

  // --- Load gait params ---
  auto gait_node = config["hexapod_gait"];
  if (gait_node) {
    hexapod_gait::GaitParams gp;
    gp.cycle_length = gait_node["cycle_length"].as<double>(48);
    gp.lift_height  = gait_node["lift_height"].as<double>(0.03);
    gp.step_time    = gait_node["step_time"].as<double>(0.05);
    gait_generator_ = hexapod_gait::GaitGenerator(gp);
    RCLCPP_INFO(get_logger(), "Gait: cycle=%.0f lift=%.3f step_time=%.3f",
      gp.cycle_length, gp.lift_height, gp.step_time);
  }

  // --- Load joint name mapping ---
  // Leg order must match gait generator: LF=0, RF=1, LM=2, RM=3, LR=4, RR=5
  const std::vector<std::string> leg_keys = {"LF", "RF", "LM", "RM", "LR", "RR"};
  auto control_node = config["hexapod_control"];
  auto joint_names_node = control_node ? control_node["joint_names"] : YAML::Node();

  all_joint_names_.clear();
  for (int leg = 0; leg < 6; ++leg) {
    if (joint_names_node && joint_names_node[leg_keys[leg]]) {
      auto names = joint_names_node[leg_keys[leg]].as<std::vector<std::string>>();
      if (names.size() == 3) {
        joint_names_[leg] = {names[0], names[1], names[2]};
      } else {
        RCLCPP_WARN(get_logger(), "Leg %s needs exactly 3 joint names, got %zu",
          leg_keys[leg].c_str(), names.size());
        joint_names_[leg] = {
          "coxa_joint_" + leg_keys[leg],
          "femur_joint_" + leg_keys[leg],
          "tibia_joint_" + leg_keys[leg]};
      }
    } else {
      joint_names_[leg] = {
        "coxa_joint_" + leg_keys[leg],
        "femur_joint_" + leg_keys[leg],
        "tibia_joint_" + leg_keys[leg]};
    }
    all_joint_names_.push_back(joint_names_[leg][0]);
    all_joint_names_.push_back(joint_names_[leg][1]);
    all_joint_names_.push_back(joint_names_[leg][2]);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu joint names", all_joint_names_.size());

  // --- Subscriber ---
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&ControlNode::cmdVelCallback, this, std::placeholders::_1));

  // --- Publisher ---
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

  // --- Timer: 500 Hz control loop ---
  control_timer_ = create_wall_timer(2ms, std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "ControlNode ready — listening on /cmd_vel");
}

void ControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_angular_ = msg->angular.z;
}

void ControlNode::controlLoop()
{
  // 1. Gait: velocity → foot positions for 6 legs
  auto feet = gait_generator_.step(cmd_vx_, cmd_vy_, cmd_angular_);

  // 2. IK: foot position → joint angles for each leg
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now();
  joint_msg.name = all_joint_names_;
  joint_msg.position.resize(18);

  for (int leg = 0; leg < 6; ++leg) {
    hexapod_ik::FootPosition foot;
    foot.x = feet[leg].x;
    foot.y = feet[leg].y;
    foot.z = feet[leg].z;

    hexapod_ik::IKResult result = ik_solvers_[leg].solve(foot);

    if (result.status == hexapod_ik::IKStatus::SUCCESS) {
      joint_msg.position[leg * 3 + 0] = result.joints.coxa;
      joint_msg.position[leg * 3 + 1] = result.joints.femur;
      joint_msg.position[leg * 3 + 2] = result.joints.tibia;
    } else {
      // Keep previous position on IK failure
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Leg %d: IK failed (status=%d)", leg, static_cast<int>(result.status));
    }
  }

  // 3. Publish joint targets
  joint_pub_->publish(joint_msg);
}

}  // namespace hexapod_control

// =============================================================================
// main
// =============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_control::ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
