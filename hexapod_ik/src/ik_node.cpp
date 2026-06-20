// =============================================================================
// ik_node.cpp — ROS 2 node wrapping IkSolver
// =============================================================================
// Subscribes to /foot_target (PointStamped), runs IK, publishes /joint_targets.
// Loads config from YAML file via yaml-cpp.

#include <hexapod_ik/ik_node.hpp>

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace hexapod_ik {

IkNode::IkNode()
: Node("ik_node")
{
  RCLCPP_INFO(get_logger(), "IkNode constructor starting");
  // --- Load YAML config file path from ROS param ---
  declare_parameter("config_file", "");
  const std::string config_file = get_parameter("config_file").as_string();

  RCLCPP_INFO(get_logger(), "config_file = '%s'", config_file.c_str());

  if (config_file.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'config_file' not set — provide path to YAML");
    return;
  }

  // --- Parse YAML ---
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse YAML: %s", e.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "YAML loaded successfully");

  // Extract IK parameters
  auto ik_node = config["hexapod_ik"];
  if (!ik_node) {
    RCLCPP_ERROR(get_logger(), "No 'hexapod_ik' key in YAML");
    return;
  }

  LegParams params;
  params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
  params.femur_length = ik_node["femur_length"].as<double>(0.0545);
  params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);

  joint_names_ = ik_node["joint_names"].as<std::vector<std::string>>(
    std::vector<std::string>{"coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR"});

  if (joint_names_.size() != 3) {
    RCLCPP_ERROR(get_logger(), "joint_names must have exactly 3 entries (coxa, femur, tibia)");
    return;
  }

  solver_ = std::make_unique<IkSolver>(params);

  RCLCPP_INFO(get_logger(),
    "IkNode ready — coxa=%.3f, femur=%.3f, tibia=%.3f m",
    params.coxa_length, params.femur_length, params.tibia_length);

  // --- Subscriber ---
  foot_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/foot_target", 10,
    std::bind(&IkNode::footTargetCallback, this, std::placeholders::_1));

  // --- Publisher ---
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

  RCLCPP_INFO(get_logger(), "Listening on /foot_target");
}

void IkNode::footTargetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  FootPosition foot;
  foot.x = msg->point.x;
  foot.y = msg->point.y;
  foot.z = msg->point.z;

  IKResult result = solver_->solve(foot);

  if (result.status == IKStatus::UNREACHABLE) {
    RCLCPP_WARN(get_logger(),
      "Target (%.3f, %.3f, %.3f) UNREACHABLE — skipping",
      foot.x, foot.y, foot.z);
    return;
  }

  if (result.status == IKStatus::ERROR) {
    RCLCPP_ERROR(get_logger(), "IK solver error for (%.3f, %.3f, %.3f)", foot.x, foot.y, foot.z);
    return;
  }

  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = now();
  joint_msg.name = joint_names_;
  joint_msg.position = {result.joints.coxa, result.joints.femur, result.joints.tibia};

  joint_pub_->publish(joint_msg);

  RCLCPP_INFO(get_logger(),
    "IK → joints: coxa=%.3f femur=%.3f tibia=%.3f rad",
    result.joints.coxa, result.joints.femur, result.joints.tibia);
}

}  // namespace hexapod_ik

// =============================================================================
// main
// =============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_ik::IkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
