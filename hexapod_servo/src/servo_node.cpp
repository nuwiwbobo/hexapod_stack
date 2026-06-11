#include <hexapod_servo/servo_node.hpp>

namespace hexapod_servo
{

ServoNode::ServoNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("servo_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Servo node created");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring servo node");

  declare_parameter("baud_rate", 1000000);
  declare_parameter("protocol_version", 1.0);
  declare_parameter("servo_count", 18);
  declare_parameter("torque_enable_reg", 24);
  declare_parameter("present_position_reg", 36);
  declare_parameter("goal_position_reg", 30);

  servo_params_.baud_rate = get_parameter("baud_rate").as_int();
  servo_params_.protocol_version = get_parameter("protocol_version").as_double();
  servo_params_.servo_count = get_parameter("servo_count").as_int();
  servo_params_.torque_enable_reg = get_parameter("torque_enable_reg").as_int();
  servo_params_.present_position_reg = get_parameter("present_position_reg").as_int();
  servo_params_.goal_position_reg = get_parameter("goal_position_reg").as_int();

  static const std::vector<std::string> servo_names = {
    "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR",
    "coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM",
    "coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF",
    "coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR",
    "coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM",
    "coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"
  };

  static const std::vector<int> servo_ids = {
    8, 10, 12, 14, 16, 18, 2, 4, 6, 7, 9, 11, 13, 15, 17, 1, 3, 5
  };

  servo_params_.joint_names = servo_names;

  for (int i = 0; i < servo_params_.servo_count; ++i) {
    ServoConfig sc;
    sc.id = servo_ids[i];
    sc.ticks = 1024;
    sc.center = 512;
    sc.max_radians = 5.236;
    sc.sign = (i < 15) ? ((i % 3 == 2) ? 1 : -1) : ((i % 3 == 2) ? -1 : 1);
    sc.offset = 0.0;
    servo_params_.servos.push_back(sc);
  }

  driver_ = std::make_unique<ServoDriver>(servo_params_);
  target_positions_.resize(servo_params_.servo_count, 0.0);

  RCLCPP_INFO(get_logger(), "Servo node configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating servo node");

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(1));

  target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_targets", rclcpp::QoS(1),
    std::bind(&ServoNode::jointTargetCallback, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1),
    std::bind(&ServoNode::emergencyStopCallback, this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&ServoNode::controlLoop, this));

  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(2),
    std::bind(&ServoNode::publishJointStates, this));

  RCLCPP_INFO(get_logger(), "Servo node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating servo node");
  control_timer_.reset();
  state_timer_.reset();
  target_sub_.reset();
  estop_sub_.reset();
  state_pub_.reset();
  if (driver_) { driver_->disableTorque(); }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up servo node");
  driver_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ServoNode::jointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (emergency_stop_) return;
  for (size_t i = 0; i < msg->position.size() && i < target_positions_.size(); ++i) {
    target_positions_[i] = msg->position[i];
  }
}

void ServoNode::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    emergency_stop_ = true;
    if (driver_) { driver_->disableTorque(); }
    RCLCPP_WARN(get_logger(), "Emergency stop activated!");
  } else {
    emergency_stop_ = false;
    if (driver_) { driver_->enableTorque(); }
    RCLCPP_INFO(get_logger(), "Emergency stop released");
  }
}

void ServoNode::controlLoop()
{
  if (emergency_stop_ || !driver_) return;
  driver_->setGoalPositions(target_positions_);
}

void ServoNode::publishJointStates()
{
  if (!state_pub_) return;

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  joint_state.name = servo_params_.joint_names;
  joint_state.position.resize(servo_params_.servo_count);

  std::vector<uint16_t> ticks;
  if (driver_ && driver_->readPresentPositions(ticks)) {
    for (int i = 0; i < servo_params_.servo_count; ++i) {
      joint_state.position[i] = driver_->tickToRadian(ticks[i], i);
    }
  } else {
    joint_state.position = target_positions_;
  }

  state_pub_->publish(joint_state);
}

}  // namespace hexapod_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hexapod_servo::ServoNode)
