#include <hexapod_control/pose_test_node.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;

namespace hexapod_control {

static const std::vector<std::string> LEG_KEYS = {"LF", "RF", "LM", "RM", "LR", "RR"};

static struct termios orig_termios;
static bool termios_saved = false;

static void restoreTerminal() {
  if (termios_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    termios_saved = false;
  }
}

PoseTestNode::PoseTestNode()
: Node("pose_test_node"),
  ik_solver_({0.044, 0.0545, 0.1019})
{
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

  auto ik_node = config["hexapod_ik"];
  hexapod_ik::LegParams params;
  if (ik_node) {
    params.coxa_length  = ik_node["coxa_length"].as<double>(0.044);
    params.femur_length = ik_node["femur_length"].as<double>(0.0545);
    params.tibia_length = ik_node["tibia_length"].as<double>(0.1019);
  }
  ik_solver_ = hexapod_ik::IkSolver(params);

  auto body_node = config["body_frame"];
  for (int i = 0; i < 6; ++i) {
    auto leg = body_node ? body_node[LEG_KEYS[i]] : YAML::Node();
    mounts_[i].mount_x = leg ? leg["mount_x"].as<double>(0.0) : 0.0;
    mounts_[i].mount_y = leg ? leg["mount_y"].as<double>(0.0) : 0.0;
    mounts_[i].coxa_angle = leg
      ? leg["coxa_angle_deg"].as<double>(90.0) * M_PI / 180.0
      : M_PI / 2.0;
  }

  auto control_node = config["hexapod_control"];
  auto jn = control_node ? control_node["joint_names"] : YAML::Node();
  for (int leg = 0; leg < 6; ++leg) {
    if (jn && jn[LEG_KEYS[leg]]) {
      auto names = jn[LEG_KEYS[leg]].as<std::vector<std::string>>();
      for (int j = 0; j < 3 && j < (int)names.size(); ++j) {
        all_joint_names_.push_back(names[j]);
      }
    } else {
      all_joint_names_.push_back("coxa_joint_" + LEG_KEYS[leg]);
      all_joint_names_.push_back("femur_joint_" + LEG_KEYS[leg]);
      all_joint_names_.push_back("tibia_joint_" + LEG_KEYS[leg]);
    }
  }

  auto pose_node = config["pose_test"];
  double foot_spread = pose_node ? pose_node["foot_spread"].as<double>(0.10) : 0.10;
  double stance_height = pose_node ? pose_node["stance_height"].as<double>(0.10) : 0.10;
  sweep_amplitude_ = pose_node ? pose_node["sweep_amplitude"].as<double>(0.4) : 0.4;
  sweep_period_ = pose_node ? pose_node["sweep_period"].as<double>(6.0) : 6.0;
  publish_rate_ = pose_node ? pose_node["publish_rate"].as<double>(20.0) : 20.0;
  
  std::string start_mode = pose_node ? pose_node["start_mode"].as<std::string>("stance") : "stance";
  if (start_mode == "coxa") mode_ = PoseMode::SWEEP_COXA;
  else if (start_mode == "femur") mode_ = PoseMode::SWEEP_FEMUR;
  else if (start_mode == "tibia") mode_ = PoseMode::SWEEP_TIBIA;
  else mode_ = PoseMode::STANCE;

  RCLCPP_INFO(get_logger(), "Computing default stance (spread=%.3fm, height=%.3fm)",
    foot_spread, stance_height);

  for (int i = 0; i < 6; ++i) {
    double alpha = mounts_[i].coxa_angle;
    body_stance_[i].x = mounts_[i].mount_x + foot_spread * std::cos(alpha);
    body_stance_[i].y = mounts_[i].mount_y + foot_spread * std::sin(alpha);
    body_stance_[i].z = -stance_height;

    hexapod_ik::FootPosition leg_foot = bodyToLeg(body_stance_[i], i);

    RCLCPP_INFO(get_logger(),
      "  %s: body=(%.3f, %.3f, %.3f) -> leg=(%.3f, %.3f, %.3f)",
      LEG_KEYS[i].c_str(),
      body_stance_[i].x, body_stance_[i].y, body_stance_[i].z,
      leg_foot.x, leg_foot.y, leg_foot.z);

    auto result = ik_solver_.solve(leg_foot);
    if (result.status == hexapod_ik::IKStatus::SUCCESS) {
      default_joints_[i] = result.joints;
      auto fk_leg = ik_solver_.forward(result.joints);
      auto fk_body = legToBody(fk_leg, i);
      RCLCPP_INFO(get_logger(),
        "    IK: coxa=%.3f femur=%.3f tibia=%.3f | FK body=(%.3f, %.3f, %.3f)",
        result.joints.coxa, result.joints.femur, result.joints.tibia,
        fk_body.x, fk_body.y, fk_body.z);
    } else {
      RCLCPP_ERROR(get_logger(), "    %s: IK FAILED", LEG_KEYS[i].c_str());
      default_joints_[i] = {0.0, -0.9, 2.0};
    }
  }

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

  int period_ms = static_cast<int>(1000.0 / publish_rate_);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PoseTestNode::controlLoop, this));

  if (isatty(STDIN_FILENO)) {
    tcgetattr(STDIN_FILENO, &orig_termios);
    termios_saved = true;
    atexit(restoreTerminal);

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  } else {
    RCLCPP_INFO(get_logger(), "No TTY detected, keyboard input disabled");
  }

  RCLCPP_INFO(get_logger(), "PoseTestNode ready");
  printStanceInfo();
}

PoseTestNode::~PoseTestNode() {
  restoreTerminal();
}

void PoseTestNode::run() {
  while (running_ && rclcpp::ok()) {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1 && c != 0) {
      handleKey(c);
    }
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void PoseTestNode::printStanceInfo() {
  printf("\n");
  printf("=== Pose Test Node ===\n");
  printf("  s = Stance (hold default position)\n");
  printf("  1 = Sweep COXA  (all legs)\n");
  printf("  2 = Sweep FEMUR (all legs)\n");
  printf("  3 = Sweep TIBIA (all legs)\n");
  printf("  q = Quit\n");
  printf("\n");
  printf("Current mode: STANCE\n");
  printf("Sweep: amplitude=%.2f rad, period=%.1f s\n", sweep_amplitude_, sweep_period_);
  printf("======================\n\n");
}

void PoseTestNode::handleKey(char key) {
  switch (key) {
    case 's': case 'S':
      mode_ = PoseMode::STANCE;
      sweep_phase_ = 0.0;
      printf("Mode: STANCE\n");
      break;
    case '1':
      mode_ = PoseMode::SWEEP_COXA;
      sweep_phase_ = 0.0;
      printf("Mode: SWEEP COXA\n");
      break;
    case '2':
      mode_ = PoseMode::SWEEP_FEMUR;
      sweep_phase_ = 0.0;
      printf("Mode: SWEEP FEMUR\n");
      break;
    case '3':
      mode_ = PoseMode::SWEEP_TIBIA;
      sweep_phase_ = 0.0;
      printf("Mode: SWEEP TIBIA\n");
      break;
    case 'q': case 'Q':
      printf("Quitting...\n");
      running_ = false;
      break;
    default:
      break;
  }
}

void PoseTestNode::controlLoop() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = all_joint_names_;
  msg.position.resize(18);

  double dt = 1.0 / publish_rate_;
  sweep_phase_ += dt;

  double sweep_offset = sweep_amplitude_ * std::sin(2.0 * M_PI * sweep_phase_ / sweep_period_);

  for (int leg = 0; leg < 6; ++leg) {
    double coxa  = default_joints_[leg].coxa;
    double femur = default_joints_[leg].femur;
    double tibia = default_joints_[leg].tibia;

    switch (mode_) {
      case PoseMode::STANCE:
        break;
      case PoseMode::SWEEP_COXA:
        coxa += sweep_offset;
        break;
      case PoseMode::SWEEP_FEMUR:
        femur += sweep_offset;
        break;
      case PoseMode::SWEEP_TIBIA:
        tibia += sweep_offset;
        break;
    }

    msg.position[leg * 3 + 0] = coxa;
    msg.position[leg * 3 + 1] = femur;
    msg.position[leg * 3 + 2] = tibia;
  }

  if (publish_count_ < 3) {
    RCLCPP_INFO(get_logger(), "Publish #%d:", publish_count_);
    for (int leg = 0; leg < 6; ++leg) {
      RCLCPP_INFO(get_logger(), "  %s: coxa=%.3f femur=%.3f tibia=%.3f",
        msg.name[leg*3].c_str(),
        msg.position[leg*3], msg.position[leg*3+1], msg.position[leg*3+2]);
    }
  }
  publish_count_++;

  joint_pub_->publish(msg);
}

hexapod_ik::FootPosition PoseTestNode::bodyToLeg(
  const hexapod_ik::FootPosition& body_foot, int leg_idx) const
{
  double alpha = mounts_[leg_idx].coxa_angle;
  double sin_a = std::sin(alpha);
  double cos_a = std::cos(alpha);

  double rel_x = body_foot.x - mounts_[leg_idx].mount_x;
  double rel_y = body_foot.y - mounts_[leg_idx].mount_y;

  hexapod_ik::FootPosition leg;
  leg.x =  rel_x * sin_a - rel_y * cos_a;
  leg.y =  rel_x * cos_a + rel_y * sin_a;
  leg.z = body_foot.z;
  return leg;
}

hexapod_ik::FootPosition PoseTestNode::legToBody(
  const hexapod_ik::FootPosition& leg_foot, int leg_idx) const
{
  double alpha = mounts_[leg_idx].coxa_angle;
  double sin_a = std::sin(alpha);
  double cos_a = std::cos(alpha);

  hexapod_ik::FootPosition body;
  body.x = leg_foot.x * sin_a + leg_foot.y * cos_a + mounts_[leg_idx].mount_x;
  body.y = -leg_foot.x * cos_a + leg_foot.y * sin_a + mounts_[leg_idx].mount_y;
  body.z = leg_foot.z;
  return body;
}

}  // namespace hexapod_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_control::PoseTestNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
