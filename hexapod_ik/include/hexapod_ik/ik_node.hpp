#ifndef HEXAPOD_IK__IK_NODE_HPP_
#define HEXAPOD_IK__IK_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <hexapod_ik/ik_solver.hpp>

namespace hexapod_ik
{

class IkNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit IkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  void feetCallback(const hexapod_msgs::msg::FeetPositions::SharedPtr msg);
  void bodyCallback(const hexapod_msgs::msg::Pose::SharedPtr msg);

  std::unique_ptr<IkSolver> solver_;
  IkParams ik_params_;
  hexapod_msgs::msg::Pose body_orientation_;

  rclcpp::Subscription<hexapod_msgs::msg::FeetPositions>::SharedPtr feet_sub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr body_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

}  // namespace hexapod_ik

#endif  // HEXAPOD_IK__IK_NODE_HPP_
