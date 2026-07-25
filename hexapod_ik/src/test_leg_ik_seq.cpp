// =============================================================================
// test_leg_ik_seq.cpp — Test each leg's IK on hardware, one at a time
// =============================================================================
// Solves IK for a stance position, publishes to /joint_targets one leg at a
// time so you can physically verify each leg moves correctly.
//
// Usage:
//   # Test one leg:
//   ros2 run hexapod_ik test_leg_ik_seq_exec --leg LF
//
//   # Test all 6 legs sequentially (4s pause between each):
//   ros2 run hexapod_ik test_leg_ik_seq_exec --all
//
//   # Custom foot position:
//   ros2 run hexapod_ik test_leg_ik_seq_exec --leg RR --fx 0.05 --fy 0.12 --fz -0.10
//
//   # Custom delay:
//   ros2 run hexapod_ik test_leg_ik_seq_exec --all --delay 5.0
//
//   # Use body-frame coordinates with mounting offsets:
//   ros2 run hexapod_ik test_leg_ik_seq_exec --all --body-frame
//
//   # Dry-run (print only, no publish):
//   ros2 run hexapod_ik test_leg_ik_seq_exec --all --dry-run
//
// Run servo_node alongside with appropriate config:
//   ros2 run hexapod_servo servo_node_exec --ros-args -p config_file:=config/test_LR.yaml

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>

#include <chrono>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Per-leg configuration
// ---------------------------------------------------------------------------
struct LegConfig {
  std::string name;
  double mount_x, mount_y;
};

static const LegConfig ALL_LEGS[] = {
  {"LF",  0.07,  0.12},
  {"RF", -0.07,  0.12},
  {"LM",  0.09,  0.00},
  {"RM", -0.09,  0.00},
  {"LR",  0.07, -0.12},
  {"RR", -0.07, -0.12},
};
static constexpr int NUM_LEGS = 6;

static const std::map<std::string, std::vector<std::string>> LEG_JOINTS = {
  {"LF", {"coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"}},
  {"RF", {"coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF"}},
  {"LM", {"coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM"}},
  {"RM", {"coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM"}},
  {"LR", {"coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR"}},
  {"RR", {"coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"}},
};

static hexapod_ik::FootPosition bodyToLeg(
  const hexapod_ik::FootPosition& bf, double mx, double my)
{
  return {bf.x - mx, bf.y - my, bf.z};
}

// ---------------------------------------------------------------------------
// Parse --key value or --flag from argv
// ---------------------------------------------------------------------------
static std::string getArg(int argc, char* argv[], const std::string& key,
                          const std::string& fallback)
{
  for (int i = 1; i < argc - 1; ++i) {
    if (argv[i] == key) return argv[i + 1];
  }
  return fallback;
}

static bool hasFlag(int argc, char* argv[], const std::string& flag)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == flag) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_leg_ik_seq");

  // --- Parse arguments ---
  std::string leg_name = getArg(argc, argv, "--leg", "");
  bool all_mode       = hasFlag(argc, argv, "--all");
  bool dry_run        = hasFlag(argc, argv, "--dry-run");
  bool body_frame     = hasFlag(argc, argv, "--body-frame");

  double fx = std::atof(getArg(argc, argv, "--fx", "0.0").c_str());
  double fy = std::atof(getArg(argc, argv, "--fy", "0.10").c_str());
  double fz = std::atof(getArg(argc, argv, "--fz", "-0.10").c_str());
  double delay_sec = std::atof(getArg(argc, argv, "--delay", "4.0").c_str());

  if (!all_mode && leg_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Usage: --leg <NAME> or --all");
    rclcpp::shutdown();
    return 1;
  }

  // --- Determine leg sequence ---
  std::vector<std::string> sequence;
  if (all_mode) {
    for (auto& l : ALL_LEGS) sequence.push_back(l.name);
  } else {
    sequence.push_back(leg_name);
  }

  // --- Create solver ---
  hexapod_ik::LegParams params{0.044, 0.0545, 0.1019};
  hexapod_ik::IkSolver solver(params);

  // --- Publisher ---
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_targets", 10);

  // Allow time for pub to connect
  std::this_thread::sleep_for(500ms);

  // --- Test each leg ---
  for (size_t i = 0; i < sequence.size(); ++i) {
    const std::string& name = sequence[i];

    // Find leg config
    const LegConfig* leg = nullptr;
    for (auto& l : ALL_LEGS) {
      if (l.name == name) { leg = &l; break; }
    }
    if (!leg) {
      RCLCPP_ERROR(node->get_logger(), "Unknown leg: %s", name.c_str());
      continue;
    }

    // Compute foot position in leg frame
    hexapod_ik::FootPosition foot{fx, fy, fz};
    if (body_frame) {
      foot = bodyToLeg(foot, leg->mount_x, leg->mount_y);
    }

    // Solve IK
    auto result = solver.solve(foot);

    RCLCPP_INFO(node->get_logger(),
      "--- [%zu/%zu] %s ---", i + 1, sequence.size(), name.c_str());
    RCLCPP_INFO(node->get_logger(),
      "  foot: (%.3f, %.3f, %.3f)  [%s frame]",
      foot.x, foot.y, foot.z,
      body_frame ? "body" : "leg");

    if (result.status != hexapod_ik::IKStatus::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "  IK: UNREACHABLE — skipping");
      continue;
    }

    RCLCPP_INFO(node->get_logger(),
      "  IK:  coxa=%.4f  femur=%.4f  tibia=%.4f rad",
      result.joints.coxa, result.joints.femur, result.joints.tibia);

    // FK verification
    auto rt = solver.forward(result.joints);
    RCLCPP_INFO(node->get_logger(),
      "  FK:  (%.4f, %.4f, %.4f)  err=%.4f",
      rt.x, rt.y, rt.z,
      std::sqrt((rt.x-foot.x)*(rt.x-foot.x) +
                (rt.y-foot.y)*(rt.y-foot.y) +
                (rt.z-foot.z)*(rt.z-foot.z)));

    // Publish
    if (!dry_run) {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = node->now();
      msg.name = LEG_JOINTS.at(name);
      msg.position = {
        result.joints.coxa,
        result.joints.femur,
        result.joints.tibia
      };
      pub->publish(msg);
      RCLCPP_INFO(node->get_logger(), "  Published to /joint_targets");
    } else {
      RCLCPP_INFO(node->get_logger(), "  [dry-run — no publish]");
    }

    // Wait before next leg (skip on last if --all)
    if (all_mode && i < sequence.size() - 1) {
      RCLCPP_INFO(node->get_logger(),
        "  Waiting %.1f s...", delay_sec);
      std::this_thread::sleep_for(
        std::chrono::duration<double>(delay_sec));
    }
  }

  if (all_mode) {
    RCLCPP_INFO(node->get_logger(),
      "All legs tested. Keeping last position for %.1f s...", delay_sec);
    std::this_thread::sleep_for(
      std::chrono::duration<double>(delay_sec));
  } else {
    // Hold position
    RCLCPP_INFO(node->get_logger(), "Holding position — Ctrl+C to exit");
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
