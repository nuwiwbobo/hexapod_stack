// =============================================================================
// test_ik_stance.cpp — Move one leg to default standing position
// =============================================================================
// Edit LEG_NAME and foot_x/foot_y/foot_z below to test each leg.
// Run: ros2 run hexapod_bringup test_ik_stance_exec

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ===================== EDIT THESE =====================
std::string LEG_NAME = "LR";       // LF, RF, LM, RM, LR, RR
double foot_x =  0.0;
double foot_y =  0.10;
double foot_z = -0.10;
// =====================================================

static const std::map<std::string, std::vector<std::string>> LEG_JOINTS = {
  {"LF", {"coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"}},
  {"RF", {"coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF"}},
  {"LM", {"coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM"}},
  {"RM", {"coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM"}},
  {"LR", {"coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR"}},
  {"RR", {"coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"}},
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ik_stance");

  // Load IK params
  std::string config_file = ament_index_cpp::get_package_share_directory("hexapod_bringup")
                          + "/config/reignblaze.yaml";
  YAML::Node config = YAML::LoadFile(config_file);
  auto ik_node = config["hexapod_ik"];

  hexapod_ik::LegParams params;
  params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
  params.femur_length = ik_node["femur_length"].as<double>(0.0545);
  params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);

  hexapod_ik::IkSolver solver(params);

  // Run IK
  hexapod_ik::FootPosition foot{foot_x, foot_y, foot_z};
  auto result = solver.solve(foot);

  if (result.status != hexapod_ik::IKStatus::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "IK failed for leg %s", LEG_NAME.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "%s IK: coxa=%.4f femur=%.4f tibia=%.4f rad",
    LEG_NAME.c_str(), result.joints.coxa, result.joints.femur, result.joints.tibia);

  // Publish
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);
  std::this_thread::sleep_for(500ms);

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node->now();
  msg.name = LEG_JOINTS.at(LEG_NAME);
  msg.position = {result.joints.coxa, result.joints.femur, result.joints.tibia};
  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Sent to /joint_targets — waiting 2s...");
  std::this_thread::sleep_for(2000ms);

  RCLCPP_INFO(node->get_logger(), "Done.");
  rclcpp::shutdown();
  return 0;
}
