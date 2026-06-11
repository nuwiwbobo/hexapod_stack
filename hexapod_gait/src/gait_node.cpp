#include <hexapod_gait/gait_node.hpp>

namespace hexapod_gait
{

GaitNode::GaitNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gait_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Gait node created");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring gait node");

  declare_parameter("cycle_length", 50);
  declare_parameter("lift_height", 0.0375);
  declare_parameter("velocity_scaling", 0.15);
  declare_parameter("low_pass_alpha", 0.05);

  gait_params_.cycle_length = get_parameter("cycle_length").as_int();
  gait_params_.lift_height = get_parameter("lift_height").as_double();
  gait_params_.velocity_scaling = get_parameter("velocity_scaling").as_double();
  gait_params_.low_pass_alpha = get_parameter("low_pass_alpha").as_double();

  engine_ = std::make_unique<GaitEngine>(gait_params_);

  RCLCPP_INFO(get_logger(), "Gait node configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating gait node");

  feet_pub_ = create_publisher<hexapod_msgs::msg::FeetPositions>(
    "/foot_positions", rclcpp::QoS(1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10),
    std::bind(&GaitNode::cmdVelCallback, this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(2),
    std::bind(&GaitNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Gait node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating gait node");
  control_timer_.reset();
  cmd_vel_sub_.reset();
  feet_pub_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GaitNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up gait node");
  engine_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GaitNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_vel_ = *msg;
}

void GaitNode::controlLoop()
{
  auto feet = engine_->compute(current_cmd_vel_);
  feet_pub_->publish(feet);
}

}  // namespace hexapod_gait

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hexapod_gait::GaitNode)
