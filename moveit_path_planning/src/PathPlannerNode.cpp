#include "../include/PathPlannerNode.hpp"
#include <chrono>


PathPlannerNode::PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group, const std::string& target_pose_topic, const std::string& target_joints_topic):
Node("path_planner_node"),
_move_group(move_group)
{
    _target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(target_pose_topic, 1,
                            std::bind(&PathPlannerNode::receiveTargetPoseCallback, this, std::placeholders::_1));
    _joint_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(target_joints_topic, 1);

}

void PathPlannerNode::receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const{
    // get trajectory plan based on target pos and publish joint targets    
    
    _move_group->setPoseTarget(*target_pose_msg);
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(_move_group->plan(msg));
        return std::make_pair(ok, msg);
    }();

    auto trajectory = plan.trajectory_;
    publishPath(trajectory);
}

void PathPlannerNode::publishPath(moveit_msgs::msg::RobotTrajectory& trajectory) const {

    for (const auto& point : trajectory.joint_trajectory.points) {
        // Create array with 6 joint positions
        std::array<double, NUM_JOINTS> joint_positions;

        for (size_t i = 0; i < NUM_JOINTS && i < point.positions.size(); ++i) {
            joint_positions[i] = point.positions[i];
        }
        
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(joint_positions.begin(), joint_positions.end());
       	_joint_pose_pub->publish(msg);
    }
}