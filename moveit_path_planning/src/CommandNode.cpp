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

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

void CommandNode::timer_callback()
{
    auto pose_msg = geometry_msgs::msg::Pose();
    pose_msg.position.x = tf.translation().x();
    pose_msg.position.y = tf.translation().y();
    pose_msg.position.z = tf.translation().z();

    Eigen::Quaterniond q(tf.rotation());
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    _target_pose_pub->publish(pose_msg);
}
