#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include "arm_msgs/srv/plan_motion.hpp"
#include "arm_msgs/action/plan_and_execute.hpp"

#define NUM_JOINTS 6

enum class ArmState {
    IDLE = 0,
    MANUAL = 1,
    IK = 2,
    PATH_PLANNING = 3,
    SCIENCE = 4
};

class PathPlannerNode: public rclcpp::Node {
private:
    ArmState _curr_state = ArmState::IDLE;
    geometry_msgs::msg::Pose::SharedPtr _curr_pose;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _target_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _curr_pose_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _arm_state_sub;


    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _joint_pose_pub;

    void receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) const;
    void updateCurrPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void updateStateCallback(std_msgs::msg::String::SharedPtr msg);

    void publishPath(moveit_msgs::msg::RobotTrajectory& trajectory) const;
    void calculatePath(const geometry_msgs::msg::Pose::SharedPtr msg) const;
    
    // move group 
    moveit::planning_interface::MoveGroupInterface* _move_group;
    rclcpp::Client<arm_msgs::srv::PlanMotion>::SharedPtr _client;
public:
    PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group, const std::string& target_pose_topic, const std::string& target_joints_topic);
};