#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include "arm_msgs/srv/plan_motion.hpp"
#include "arm_msgs/action/plan_and_execute.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#define NUM_JOINTS 6

class CommandNode : public rclcpp::Node
{
private:
//   ArmState _curr_state = ArmState::IDLE;
//   std::shared_ptr<geometry_msgs::msg::Pose> _curr_pose;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _target_pose_pub;

  // move group
  moveit::planning_interface::MoveGroupInterface * _move_group;
  rclcpp::Client<arm_msgs::srv::PlanMotion>::SharedPtr _client;
  // robot model for move group
  moveit::core::RobotModelPtr robot_model;
  const moveit::core::JointModelGroup * jmg;
  moveit::core::RobotStatePtr robot_state;

  void timer_callback();

public:
  CommandNode(moveit::planning_interface::MoveGroupInterface * move_group);
};
