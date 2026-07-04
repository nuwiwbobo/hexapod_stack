// =============================================================================
// test_all_legs_stance.cpp — Move ALL 6 legs to standing position
// =============================================================================
// Run: ros2 run hexapod_ik test_all_legs_stance_exec
//
// Solves IK for standing position (foot_x, foot_y, foot_z) for each leg,
// then publishes all 18 joints to /joint_targets.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>

#include <chrono>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ===================== EDIT THESE =====================
double foot_x =  0.0;
double foot_y =  0.10;
double foot_z = -0.10;
// =====================================================

static const std::vector<std::string> LEG_ORDER = {"LR", "LM", "LF", "RR", "RM", "RF"};

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
  auto node = rclcpp::Node::make_shared("test_all_legs_stance");

  hexapod_ik::LegParams params{0.044, 0.0545, 0.1019};
  hexapod_ik::IkSolver solver(params);

  hexapod_ik::FootPosition foot{foot_x, foot_y, foot_z};

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node->now();

  for (const auto& leg : LEG_ORDER) {
    auto result = solver.solve(foot);

    if (result.status != hexapod_ik::IKStatus::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "IK failed for leg %s", leg.c_str());
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(node->get_logger(), "%s: coxa=%.4f femur=%.4f tibia=%.4f",
      leg.c_str(), result.joints.coxa, result.joints.femur, result.joints.tibia);

    for (const auto& name : LEG_JOINTS.at(leg)) {
      msg.name.push_back(name);
    }
    msg.position.push_back(result.joints.coxa);
    msg.position.push_back(result.joints.femur);
    msg.position.push_back(result.joints.tibia);
  }

  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);
  std::this_thread::sleep_for(500ms);
  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Sent all 18 joints to /joint_targets — waiting 2s...");
  std::this_thread::sleep_for(2000ms);

  RCLCPP_INFO(node->get_logger(), "Done.");
  rclcpp::shutdown();
  return 0;
}
