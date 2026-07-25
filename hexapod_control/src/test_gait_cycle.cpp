// =============================================================================
// test_gait_cycle.cpp — Run the wave gait + IK on hardware
// =============================================================================
// Steps through the full 48-step wave gait cycle, computes IK for all 6 legs,
// and publishes /joint_targets so servo_node moves the real servos.
//
// Usage:
//   # Walk forward at 0.05 m/s:
//   ros2 run hexapod_control test_gait_cycle_exec --vx 0.05
//
//   # Walk sideways + turn:
//   ros2 run hexapod_control test_gait_cycle_exec --vx 0.03 --vy 0.02 --va 0.1
//
//   # Run for 3 full cycles then exit:
//   ros2 run hexapod_control test_gait_cycle_exec --vx 0.05 --cycles 3
//
//   # Faster gait (gait generator runs at 20 Hz by default):
//   ros2 run hexapod_control test_gait_cycle_exec --vx 0.05 --hz 50
//
//   # Dry-run (print only, no servo commands):
//   ros2 run hexapod_control test_gait_cycle_exec --vx 0.05 --dry-run
//
// Leg order: LF=0, RF=1, LM=2, RM=3, LR=4, RR=5
// Wave pattern: LF → RF → LM → RM → LR → RR (8 steps each)
//
// Run alongside: ros2 launch hexapod_bringup reignblaze.launch.py  (or servo_node)
//
// Note: gait.step() is called at the timer rate (default 20 Hz). The gait
// generator's step_time=0.05 means each step represents 50ms of movement,
// so running at 20 Hz gives vx in m/s directly.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>
#include <hexapod_gait/gait_generator.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// Leg order (matches gait generator)
static const std::vector<std::string> LEG_KEYS = {"LF", "RF", "LM", "RM", "LR", "RR"};
static const std::vector<std::string> JOINT_PARTS = {"coxa", "femur", "tibia"};

// Build flat joint name list [coxa_LF, femur_LF, tibia_LF, coxa_RF, ...]
static std::vector<std::string> makeJointNames() {
  std::vector<std::string> names;
  for (auto& leg : LEG_KEYS) {
    for (auto& part : JOINT_PARTS) {
      names.push_back(part + "_joint_" + leg);
    }
  }
  return names;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_gait_cycle");

  // --- Parse command-line args ---
  double vx  = 0.05;
  double vy  = 0.0;
  double va  = 0.0;
  double hz  = 20.0;
  int cycles = 0;   // 0 = run forever
  bool dry_run = false;

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--vx"  && i + 1 < argc) vx  = std::atof(argv[++i]);
    if (a == "--vy"  && i + 1 < argc) vy  = std::atof(argv[++i]);
    if (a == "--va"  && i + 1 < argc) va  = std::atof(argv[++i]);
    if (a == "--hz"  && i + 1 < argc) hz  = std::atof(argv[++i]);
    if (a == "--cycles" && i + 1 < argc) cycles = std::atoi(argv[++i]);
    if (a == "--dry-run") dry_run = true;
  }

  // --- Gait generator ---
  hexapod_gait::GaitParams gp;
  gp.cycle_length = 48;
  gp.lift_height  = 0.03;
  gp.step_time    = 0.05;
  hexapod_gait::GaitGenerator gait(gp);

  // --- 6 IK solvers ---
  hexapod_ik::LegParams lp{0.044, 0.0545, 0.1019};
  std::array<hexapod_ik::IkSolver, 6> iks = {
    hexapod_ik::IkSolver(lp), hexapod_ik::IkSolver(lp),
    hexapod_ik::IkSolver(lp), hexapod_ik::IkSolver(lp),
    hexapod_ik::IkSolver(lp), hexapod_ik::IkSolver(lp),
  };

  // --- Joint names ---
  auto joint_names = makeJointNames();

  // --- Publisher ---
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_targets", 10);

  // --- State ---
  int step = 0;
  int max_steps = (cycles > 0) ? cycles * 48 : 0;
  auto period = std::chrono::duration<double>(1.0 / hz);

  // --- Timer ---
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [&]() {
      auto feet = gait.step(vx, vy, va);
      step++;

      sensor_msgs::msg::JointState msg;
      msg.header.stamp = node->now();
      msg.name    = joint_names;
      msg.position.resize(18);

      bool unreachable = false;
      for (int leg = 0; leg < 6; ++leg) {
        auto fp = hexapod_ik::FootPosition{feet[leg].x, feet[leg].y, feet[leg].z};
        auto r = iks[leg].solve(fp);
        if (r.status == hexapod_ik::IKStatus::SUCCESS) {
          msg.position[leg * 3 + 0] = r.joints.coxa;
          msg.position[leg * 3 + 1] = r.joints.femur;
          msg.position[leg * 3 + 2] = r.joints.tibia;
        } else {
          unreachable = true;
        }
      }

      if (!dry_run) pub->publish(msg);

      // --- Logging ---
      // Every 8 steps (at the start of each leg's swing), show a status line
      if (step % 8 == 1 || step <= 2) {
        int swing_leg = ((step - 1) / 8) % 6;
        double lift_z = feet[swing_leg].z;

        std::stringstream ss;
        ss << "Step " << std::setw(3) << step
           << " | swing " << LEG_KEYS[swing_leg]
           << " (z=" << std::fixed << std::setprecision(3) << lift_z << ")";

        // Show first leg's full foot state
        auto& f0 = feet[0];
        ss << " | LF foot (" << std::fixed << std::setprecision(3)
           << f0.x << ", " << f0.y << ", " << f0.z << ")";

        if (unreachable) ss << " [UNREACHABLE]";
        RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
      }

      // Exit after N cycles
      if (max_steps > 0 && step >= max_steps) {
        RCLCPP_INFO(node->get_logger(),
          "Completed %d cycles (%d steps).", cycles, step);
        rclcpp::shutdown();
      }
    });

  RCLCPP_INFO(node->get_logger(),
    "Gait cycle test: vx=%.2f vy=%.2f va=%.2f at %.0f Hz%s",
    vx, vy, va, hz, dry_run ? " (dry-run)" : "");

  RCLCPP_INFO(node->get_logger(),
    "Gait: 48-step wave, lift=%.3f m, step_time=%.2f s",
    gp.lift_height, gp.step_time);

  if (cycles > 0) {
    RCLCPP_INFO(node->get_logger(),
      "Running %d cycles (%d steps) — Ctrl+C to cancel",
      cycles, max_steps);
  } else {
    RCLCPP_INFO(node->get_logger(),
      "Running continuously — Ctrl+C to stop");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
