#ifndef HEXAPOD_IK__IK_NODE_HPP_
#define HEXAPOD_IK__IK_NODE_HPP_

// =============================================================================
// ik_node.hpp — ROS 2 node wrapping IkSolver
// =============================================================================
// Subscribes to /foot_target (geometry_msgs/PointStamped) — foot position in
// body frame (meters). Runs analytical IK, publishes /joint_targets (JointState).
//
// Parameters (from YAML):
//   hexapod_ik.coxa_length, femur_length, tibia_length
//   hexapod_ik.joint_names — ["coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR"]

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hexapod_ik/ik_solver.hpp>

#include <memory>
#include <string>
#include <vector>

namespace hexapod_ik {

class IkNode : public rclcpp::Node {
public:
  IkNode();

private:
  void footTargetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  std::unique_ptr<IkSolver> solver_;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr foot_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

}  // namespace hexapod_ik

#endif  // HEXAPOD_IK__IK_NODE_HPP_
