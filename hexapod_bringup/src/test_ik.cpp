// =============================================================================
// test_ik.cpp — CLI tool: send one foot target to one leg's IK solver
// =============================================================================
// Usage: ros2 run hexapod_bringup test_ik_exec -- --leg LR 0.0 0.1 -0.1
//   --leg LF|RF|LM|RM|LR|RR
//   x y z  (foot position in meters)
//
// Runs IK, publishes to /joint_targets, waits for servo response, then exits.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <csignal>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

static std::map<std::string, std::vector<std::string>> LEG_JOINTS = {
  {"LF", {"coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"}},
  {"RF", {"coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF"}},
  {"LM", {"coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM"}},
  {"RM", {"coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM"}},
  {"LR", {"coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR"}},
  {"RR", {"coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"}},
};

int main(int argc, char* argv[])
{
  // Parse args: --leg NAME x y z
  if (argc < 6) {
    fprintf(stderr, "Usage: test_ik_exec --leg LF|RF|LM|RM|LR|RR x y z\n");
    return 1;
  }

  std::string leg_name;
  double x, y, z;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--leg" && i + 1 < argc) {
      leg_name = argv[++i];
    }
  }

  if (leg_name.empty() || LEG_JOINTS.find(leg_name) == LEG_JOINTS.end()) {
    fprintf(stderr, "Error: --leg must be one of LF, RF, LM, RM, LR, RR\n");
    return 1;
  }

  // Get x y z from remaining positional args
  std::vector<double> vals;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--leg") { ++i; continue; }
    vals.push_back(std::atof(argv[i]));
  }

  if (vals.size() != 3) {
    fprintf(stderr, "Error: need exactly 3 positional args (x y z)\n");
    return 1;
  }
  x = vals[0]; y = vals[1]; z = vals[2];

  // Init ROS
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ik_node");

  // Load IK params from config
  auto config_file = node->declare_parameter("config_file", "");
  if (config_file.empty()) {
    // Try loading from share directory
    config_file = ament_index_cpp::get_package_share_directory("hexapod_bringup")
                + "/config/reignblaze.yaml";
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const YAML::Exception& e) {
    fprintf(stderr, "Failed to load YAML: %s\n", e.what());
    return 1;
  }

  auto ik_node = config["hexapod_ik"];
  hexapod_ik::LegParams params;
  if (ik_node) {
    params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
    params.femur_length = ik_node["femur_length"].as<double>(0.0545);
    params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);
  }

  hexapod_ik::IkSolver solver(params);

  // Run IK
  hexapod_ik::FootPosition foot;
  foot.x = x; foot.y = y; foot.z = z;

  RCLCPP_INFO(node->get_logger(), "Running IK for leg %s: (%.3f, %.3f, %.3f)",
    leg_name.c_str(), x, y, z);

  auto result = solver.solve(foot);

  if (result.status == hexapod_ik::IKStatus::UNREACHABLE) {
    RCLCPP_ERROR(node->get_logger(),
      "UNREACHABLE: (%.3f, %.3f, %.3f) is out of reach for leg %s",
      x, y, z, leg_name.c_str());
    return 1;
  }

  if (result.status == hexapod_ik::IKStatus::ERROR) {
    RCLCPP_ERROR(node->get_logger(), "IK solver error");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
    "IK result: coxa=%.4f femur=%.4f tibia=%.4f rad",
    result.joints.coxa, result.joints.femur, result.joints.tibia);

  // Publish to /joint_targets
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

  // Wait for subscriber connections
  std::this_thread::sleep_for(500ms);

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node->now();
  msg.name = LEG_JOINTS[leg_name];
  msg.position = {result.joints.coxa, result.joints.femur, result.joints.tibia};

  pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "Published /joint_targets for %s — waiting 2s for servo response...",
    leg_name.c_str());

  // Spin briefly to receive /joint_states feedback
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(2000ms);

  RCLCPP_INFO(node->get_logger(), "Done.");
  rclcpp::shutdown();
  return 0;
}
