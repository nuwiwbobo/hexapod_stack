#include <hexapod_control/control_node.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace hexapod_control
{

ControlNode::ControlNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("control_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Control node created");
}

std::vector<std::string> ControlNode::getJointNames()
{
  return {
    "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR",
    "coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM",
    "coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF",
    "coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR",
    "coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM",
    "coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"
  };
}

hexapod_msgs::msg::RPY ControlNode::getBodyOrientation() const
{
  return body_orientation_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControlNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring control node");
  last_odom_time_ = get_clock()->now();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControlNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating control node");

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(50));
  body_pub_ = create_publisher<hexapod_msgs::msg::RPY>("/body_orientation", rclcpp::QoS(1));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(1),
    std::bind(&ControlNode::jointStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Control node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControlNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating control node");
  joint_state_sub_.reset();
  odom_pub_.reset();
  body_pub_.reset();
  tf_broadcaster_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up control node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ControlNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  body_orientation_.roll = 0.0;
  body_orientation_.pitch = 0.0;
  body_orientation_.yaw = 0.0;
  body_pub_->publish(body_orientation_);

  auto now = get_clock()->now();
  double dt = (now - last_odom_time_).seconds();
  last_odom_time_ = now;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = pose_x_;
  odom.pose.pose.position.y = pose_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.z = std::sin(pose_th_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(pose_th_ / 2.0);
  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = pose_x_;
  odom_tf.transform.translation.y = pose_y_;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(odom_tf);
}

}  // namespace hexapod_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hexapod_control::ControlNode)
