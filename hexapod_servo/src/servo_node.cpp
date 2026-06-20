// =============================================================================
// servo_node.cpp — ROS 2 node wrapping ServoDriver
// =============================================================================
// Subscribes to /joint_targets (JointState), sends radians to servos.
// Publishes /joint_states (JointState) on 10 Hz timer — reads actual positions.
//
// Loads config from YAML file via yaml-cpp (not ROS param server).

#include <hexapod_servo/servo_node.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace hexapod_servo {

ServoNode::ServoNode()
: Node("servo_node")
{
  // --- Load YAML config file path from ROS param ---
  declare_parameter("config_file", "");
  const std::string config_file = get_parameter("config_file").as_string();

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

  // Extract servo parameters
  const std::string port = config["hexapod_servo"]["port"].as<std::string>("/dev/ttyUSB0");
  const int baud = config["hexapod_servo"]["baud_rate"].as<int>(1000000);

  auto servos_node = config["hexapod_servo"]["servos"];
  if (!servos_node) {
    RCLCPP_ERROR(get_logger(), "No 'servos' key in YAML");
    return;
  }

  std::vector<ServoConfig> configs;
  joint_names_.clear();

  for (auto it = servos_node.begin(); it != servos_node.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    auto servo = it->second;

    ServoConfig cfg;
    cfg.id          = servo["id"].as<int>();
    cfg.type        = servo["type"].as<std::string>("AX-18A");
    cfg.ticks       = servo["ticks"].as<int>(1024);
    cfg.center      = servo["center"].as<int>(512);
    cfg.max_radians = servo["max_radians"].as<double>(5.236);
    cfg.sign        = servo["sign"].as<int>(1);
    cfg.offset      = servo["offset"].as<double>(0.0);
    configs.push_back(cfg);
    joint_names_.push_back(name);

    RCLCPP_INFO(get_logger(), "  Loaded servo: %s (id=%d, sign=%d, offset=%.5f)",
      name.c_str(), cfg.id, cfg.sign, cfg.offset);
  }

  if (configs.empty()) {
    RCLCPP_ERROR(get_logger(), "No servos configured in YAML");
    return;
  }

  // --- Create driver and open port ---
  driver_ = std::make_unique<ServoDriver>(configs);

  // Initialize last targets to center (0 radians) for all servos
  last_targets_.assign(configs.size(), 0.0);

  RCLCPP_INFO(get_logger(), "Opening port %s @ %d baud (%zu servos)",
    port.c_str(), baud, driver_->getServoCount());

  if (!driver_->openPort(port, baud)) {
    RCLCPP_ERROR(get_logger(), "Failed to open port %s", port.c_str());
    return;
  }
  port_open_ = true;

  if (!driver_->enableTorque(100, 300)) {
    RCLCPP_WARN(get_logger(), "Torque enable returned errors (some servos may not respond)");
  }

  // --- Subscriber ---
  target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_targets", 10,
    std::bind(&ServoNode::jointTargetCallback, this, std::placeholders::_1));

  // --- Publisher ---
  state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // --- Timer: read positions at 10 Hz ---
  publish_timer_ = create_wall_timer(100ms, [this]() {
    if (!port_open_ || !driver_) return;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name = joint_names_;

    auto positions = driver_->readPositions();
    msg.position.assign(positions.begin(), positions.end());

    state_pub_->publish(msg);
  });

  RCLCPP_INFO(get_logger(), "ServoNode ready — listening on /joint_targets");
}

void ServoNode::jointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!port_open_ || !driver_) return;

  // Start from last known positions (keeps unchanged joints at their current target)
  std::vector<double> targets = last_targets_;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
    if (it == joint_names_.end()) continue;

    size_t servo_idx = static_cast<size_t>(std::distance(joint_names_.begin(), it));

    if (i >= msg->position.size()) continue;

    targets[servo_idx] = msg->position[i];
  }

  // Single sync write — all 18 servos in one bus packet
  if (!driver_->setGoalPositions(targets)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Sync write failed (some servos may not respond)");
  }

  last_targets_ = targets;
}

}  // namespace hexapod_servo

// =============================================================================
// main
// =============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_servo::ServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
