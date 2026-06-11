#ifndef HEXAPOD_CONTROL__CONTROL_NODE_HPP_
#define HEXAPOD_CONTROL__CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hexapod_msgs/msg/rpy.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace hexapod_control
{

class ControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  static std::vector<std::string> getJointNames();
  hexapod_msgs::msg::RPY getBodyOrientation() const;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  hexapod_msgs::msg::RPY body_orientation_;
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_th_ = 0.0;
  rclcpp::Time last_odom_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<hexapod_msgs::msg::RPY>::SharedPtr body_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace hexapod_control

#endif  // HEXAPOD_CONTROL__CONTROL_NODE_HPP_
