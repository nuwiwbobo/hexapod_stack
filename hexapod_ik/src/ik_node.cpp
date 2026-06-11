#include <hexapod_ik/ik_node.hpp>
#include <vector>

namespace hexapod_ik
{

IkNode::IkNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ik_node", "", options)
{
  RCLCPP_INFO(get_logger(), "IK node created");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
IkNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring IK node");

  declare_parameter("number_of_legs", 6);
  declare_parameter("coxa_length", 0.044);
  declare_parameter("femur_length", 0.0545);
  declare_parameter("tibia_length", 0.1019);
  declare_parameter("tarsus_length", 0.0);

  ik_params_.number_of_legs = get_parameter("number_of_legs").as_int();
  ik_params_.coxa_length = get_parameter("coxa_length").as_double();
  ik_params_.femur_length = get_parameter("femur_length").as_double();
  ik_params_.tibia_length = get_parameter("tibia_length").as_double();
  ik_params_.tarsus_length = get_parameter("tarsus_length").as_double();

  ik_params_.init_coxa_angle = get_parameter("init_coxa_angle").as_double_array();
  ik_params_.coxa_to_center_x = get_parameter("coxa_to_center_x").as_double_array();
  ik_params_.coxa_to_center_y = get_parameter("coxa_to_center_y").as_double_array();
  ik_params_.init_foot_pos_x = get_parameter("init_foot_pos_x").as_double_array();
  ik_params_.init_foot_pos_y = get_parameter("init_foot_pos_y").as_double_array();
  ik_params_.init_foot_pos_z = get_parameter("init_foot_pos_z").as_double_array();

  solver_ = std::make_unique<IkSolver>(ik_params_);

  RCLCPP_INFO(get_logger(), "IK node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
IkNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating IK node");

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_targets", rclcpp::QoS(1));

  feet_sub_ = create_subscription<hexapod_msgs::msg::FeetPositions>(
    "/foot_positions", rclcpp::QoS(1),
    std::bind(&IkNode::feetCallback, this, std::placeholders::_1));

  body_sub_ = create_subscription<hexapod_msgs::msg::Pose>(
    "/body_orientation", rclcpp::QoS(1),
    std::bind(&IkNode::bodyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "IK node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
IkNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating IK node");
  joint_pub_.reset();
  feet_sub_.reset();
  body_sub_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
IkNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up IK node");
  solver_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void IkNode::bodyCallback(const hexapod_msgs::msg::Pose::SharedPtr msg)
{
  body_orientation_ = *msg;
}

void IkNode::feetCallback(const hexapod_msgs::msg::FeetPositions::SharedPtr msg)
{
  auto result = solver_->calculateIK(*msg, body_orientation_);

  if (!result.success) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK solver: target unreachable");
    return;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();

  static const std::vector<std::string> joint_names = {
    "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR",
    "coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM",
    "coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF",
    "coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR",
    "coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM",
    "coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"
  };

  joint_state.name = joint_names;
  joint_state.position.resize(18);

  for (int leg = 0; leg < 6; ++leg) {
    joint_state.position[leg * 3 + 0] = result.joints.leg[leg].coxa;
    joint_state.position[leg * 3 + 1] = result.joints.leg[leg].femur;
    joint_state.position[leg * 3 + 2] = result.joints.leg[leg].tibia;
  }

  joint_pub_->publish(joint_state);
}

}  // namespace hexapod_ik

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hexapod_ik::IkNode)
