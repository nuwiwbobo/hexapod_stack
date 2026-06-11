#ifndef HEXAPOD_GAIT__GAIT_NODE_HPP_
#define HEXAPOD_GAIT__GAIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <hexapod_gait/gait_engine.hpp>

namespace hexapod_gait
{

class GaitNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GaitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlLoop();

  std::unique_ptr<GaitEngine> engine_;
  GaitParams gait_params_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<hexapod_msgs::msg::FeetPositions>::SharedPtr feet_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  geometry_msgs::msg::Twist current_cmd_vel_;
};

}  // namespace hexapod_gait

#endif  // HEXAPOD_GAIT__GAIT_NODE_HPP_
