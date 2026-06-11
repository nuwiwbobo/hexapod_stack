#ifndef HEXAPOD_SERVO__SERVO_NODE_HPP_
#define HEXAPOD_SERVO__SERVO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <hexapod_servo/servo_driver.hpp>

namespace hexapod_servo
{

class ServoNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  void jointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();
  void publishJointStates();

  std::unique_ptr<ServoDriver> driver_;
  ServoParams servo_params_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::vector<double> target_positions_;
  bool emergency_stop_ = false;
};

}  // namespace hexapod_servo

#endif  // HEXAPOD_SERVO__SERVO_NODE_HPP_
