#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#define NUM_JOINTS 6

class PathPlannerNode: public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _target_pose_sub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _joint_pose_pub;

    void receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) const;
    void publishPath(moveit_msgs::msg::RobotTrajectory& trajectory) const;

    // move group 
    moveit::planning_interface::MoveGroupInterface* _move_group;
public:
    PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group, const std::string& target_pose_topic, const std::string& target_joints_topic);
};