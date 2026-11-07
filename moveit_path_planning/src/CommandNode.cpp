#include "../include/CommandNode.hpp"
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "command_node_move_group",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const std::string planning_group = "rover_arm";
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, planning_group);
  auto command_node = std::make_shared<CommandNode>(&move_group_interface);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(command_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

// ---------------------------------------------------------------------------
// CommandNode
// ---------------------------------------------------------------------------

CommandNode::CommandNode(
    moveit::planning_interface::MoveGroupInterface * move_group
): Node("command_node"),
  _move_group(move_group)
{
    _target_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        "arm_target_pose",
        1);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CommandNode::timer_callback, this));
};

void CommandNode::timer_callback()
{
    auto pose_msg = geometry_msgs::msg::Pose();
    pose_msg.position.x = 0.4;
    pose_msg.position.y = 0.4;
    pose_msg.position.z = 0.5;

    pose_msg.orientation.x = 0.0;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_msg.orientation.w = 1.0;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    _target_pose_pub->publish(pose_msg);
}
