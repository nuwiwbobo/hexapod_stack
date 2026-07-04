// =============================================================================
// read_positions.cpp — Print current position of all servos from /joint_states
// =============================================================================
// Run: ros2 run hexapod_servo read_positions_exec
//
// Requires servo_node to be running (publishes /joint_states).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("read_positions");

  // Load config to get servo IDs
  std::string config_file = ament_index_cpp::get_package_share_directory("hexapod_bringup")
                          + "/config/reignblaze.yaml";
  YAML::Node config = YAML::LoadFile(config_file);

  std::map<std::string, int> joint_to_id;
  auto servos = config["hexapod_servo"]["servos"];
  for (auto it = servos.begin(); it != servos.end(); ++it) {
    std::string name = it->first.as<std::string>();
    int id = it->second["id"].as<int>();
    joint_to_id[name] = id;
  }

  // Subscribe to /joint_states
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  bool received = false;

  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_names = msg->name;
      positions = msg->position;
      received = true;
    });

  // Wait for first message
  std::cout << "Waiting for /joint_states..." << std::endl;
  auto start = node->now();
  while (rclcpp::ok() && !received && (node->now() - start).seconds() < 5.0) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  if (!received) {
    std::cerr << "No /joint_states received (is servo_node running?)" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Print positions
  std::cout << "\n=== Servo Positions ===" << std::endl;
  std::cout << std::left << std::setw(6) << "ID"
            << std::left << std::setw(20) << "Joint"
            << std::right << std::setw(10) << "Rad"
            << std::setw(10) << "Deg" << std::endl;
  std::cout << std::string(46, '-') << std::endl;

  for (size_t i = 0; i < joint_names.size(); ++i) {
    double rad = positions[i];
    double deg = rad * 180.0 / M_PI;
    int id = joint_to_id.count(joint_names[i]) ? joint_to_id[joint_names[i]] : -1;
    std::cout << std::left << std::setw(6) << id
              << std::left << std::setw(20) << joint_names[i]
              << std::right << std::fixed << std::setprecision(4)
              << std::setw(10) << rad
              << std::setw(10) << deg << std::endl;
  }

  std::cout << "\nTotal: " << joint_names.size() << " joints" << std::endl;

  rclcpp::shutdown();
  return 0;
}
