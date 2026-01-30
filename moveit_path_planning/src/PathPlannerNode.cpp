#include "../include/PathPlannerNode.hpp"
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <sstream> // Required for std::stringstream
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>

using namespace std::chrono_literals;
PathPlannerNode::PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group, const std::string& target_pose_topic, const std::string& target_joints_topic):
Node("path_planner_node"),
_move_group(move_group)
{
    _target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("arm_target_pose", 1,
                            std::bind(&PathPlannerNode::receiveTargetPoseCallback, this, std::placeholders::_1));
    _arm_state_sub = this->create_subscription<std_msgs::msg::String>("arm_state", 1,
                            std::bind(&PathPlannerNode::updateStateCallback, this, std::placeholders::_1));
    _joint_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("arm_curr_angles", 1,
                            std::bind(&PathPlannerNode::joint_callback, this, std::placeholders::_1));


    _joint_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_ik_target_joints", 1);
    _rviz_joint_pose_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    _pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("arm_fk_pose", 1);

    _joint_path_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_path_joints", 1);

    moveit::core::RobotModelConstPtr robot_model_;
    robot_model_ = _move_group->getRobotModel();
    jmg = robot_model_->getJointModelGroup(_move_group->getName());
    robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state->setToDefaultValues();

    //_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void PathPlannerNode::receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const{
    // get trajectory plan based on target pos and publish joint targets 
    switch(_curr_state){
        case ArmState::IK: {
            // incremental update
            // check if still required, utils map_input_to_ik seems to handle it
            calculateIK(target_pose_msg);
            break;
        }
        case ArmState::PATH_PLANNING: {
            calculatePath(target_pose_msg);
            break;
        }
        default:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning not configured for state");
        break;
    }
}


// Helper: convert geometry_msgs::Pose -> Eigen::Isometry3d
Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
    eigen_pose.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    eigen_pose.linear() = q.toRotationMatrix();
    return eigen_pose;
}
void PathPlannerNode::calculateIK(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const
{
    RCLCPP_INFO(
    rclcpp::get_logger("pose_logger"),
    "Position: x=%f y=%f z=%f | Orientation: x=%f y=%f z=%f w=%f",
    target_pose_msg->position.x,
    target_pose_msg->position.y,
    target_pose_msg->position.z,
    target_pose_msg->orientation.x,
    target_pose_msg->orientation.y,
    target_pose_msg->orientation.z,
    target_pose_msg->orientation.w
  );

    if (!_move_group) {
        RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized");
        return;
    }
    
    moveit::core::RobotStatePtr current_state = robot_state;
    
    // Get joint model group
    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(_move_group->getName());
    if (!joint_model_group) {
        RCLCPP_ERROR(get_logger(), "JointModelGroup not found!");
        return;
    }
    
    // Convert Pose -> Eigen::Isometry3d
    Eigen::Isometry3d target_pose_eigen = poseMsgToEigen(*target_pose_msg);
    
    // Get current planning scene for our arm (necessary to properly calculate collisions)
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_));

    // Create the actual callback to check the collisions
    moveit::core::GroupStateValidityCallbackFn constraint_fn = 
        [planning_scene](moveit::core::RobotState* robot_state, 
                     const moveit::core::JointModelGroup* joint_group, 
                     const double* joint_group_variable_values) 
    {
        robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
        robot_state->update();

        return !planning_scene->isStateColliding(*robot_state, joint_group->getName());
    };

    // Solve IK using the new callback
    bool found_ik = current_state->setFromIK(
        joint_model_group,
        target_pose_eigen,
        //_move_group->getEndEffectorLink(),
        0.1,
        constraint_fn
        //kinematics::KinematicsQueryOptions()
    );
    
    if (!found_ik) {
        RCLCPP_WARN(get_logger(), "IK solution not found for target pose");
        return;
    }
    
    // Extract joint positions
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    RCLCPP_INFO(get_logger(), "IK solution found:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Joint %zu: %.4f", i, joint_values[i]);
    }

    // Publish joint positions
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(joint_values.begin(), joint_values.end());
    _joint_pose_pub->publish(msg);

}

// In your joint_callback function - add this at the end:
void PathPlannerNode::joint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::stringstream ss;
    for (size_t i = 0; i < msg->data.size(); ++i) {
        ss << msg->data[i];
        if (i < msg->data.size() - 1) {
            ss << ", ";
        }
    }
    
    // Convert float -> double
    std::vector<double> joint_positions(msg->data.begin(), msg->data.end());

    // Set joint positions
    robot_state->setJointGroupPositions(jmg, joint_positions);
    robot_state->update();

    // Compute FK
    Eigen::Isometry3d tf = robot_state->getGlobalLinkTransform("link_6");

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = tf.translation().x();
    pose_msg.position.y = tf.translation().y();
    pose_msg.position.z = tf.translation().z();

    Eigen::Quaterniond q(tf.rotation());
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    _pose_pub->publish(pose_msg);
    _curr_pose = std::make_shared<geometry_msgs::msg::Pose>(pose_msg);

    // ✅ ADD THIS: Republish as JointState for MoveIt
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = "";
    
    // Get joint names from your joint model group
    joint_state_msg.name = jmg->getVariableNames();
    joint_state_msg.position = joint_positions;
    
    //_joint_state_pub->publish(joint_state_msg);
}

void PathPlannerNode::calculatePath(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sub callback");

    _move_group->setPoseTarget(*target_pose_msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    bool success = static_cast<bool>(_move_group->plan(plan_msg));

    if (!success) {
        RCLCPP_WARN(get_logger(), "Planning failed for the given target pose");
        return;
    }

    _move_group->execute(plan_msg);
    RCLCPP_INFO(get_logger(), "Planning succeeded! Publishing trajectory...");

    publishPath(plan_msg.trajectory_);
}

void PathPlannerNode::updateCurrPoseCallback(geometry_msgs::msg::Pose::SharedPtr curr_pose_msg) {
    _curr_pose = curr_pose_msg;
}

void PathPlannerNode::updateStateCallback(std_msgs::msg::String::SharedPtr curr_state_msg){
    if(curr_state_msg->data == "IDLE"){
        _curr_state = ArmState::IDLE;
    } else if(curr_state_msg->data == "MANUAL"){
        _curr_state = ArmState::MANUAL;
    } else if(curr_state_msg->data == "IK"){
        _curr_state = ArmState::IK;
    } else if(curr_state_msg->data== "PATH_PLANNING"){
        _curr_state = ArmState::PATH_PLANNING;
    } else if(curr_state_msg->data == "SCIENCE"){
        _curr_state = ArmState::SCIENCE;
    } 
}


void PathPlannerNode::publishPath(moveit_msgs::msg::RobotTrajectory& trajectory) const{

    for (const auto& point : trajectory.joint_trajectory.points) {
        // Create array with 6 joint positions
        std::array<double, NUM_JOINTS> joint_positions;

        for (size_t i = 0; i < NUM_JOINTS && i < point.positions.size(); ++i) {
            joint_positions[i] = point.positions[i];
        }
        
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(joint_positions.begin(), joint_positions.end());
       	_joint_path_pub->publish(msg);
    }
    
}