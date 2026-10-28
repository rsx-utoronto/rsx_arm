#include "../include/PathPlannerNode.hpp"
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
// PathPlannerNode
// ---------------------------------------------------------------------------

PathPlannerNode::PathPlannerNode(
  moveit::planning_interface::MoveGroupInterface * move_group,
  const std::string & target_pose_topic,
  const std::string & target_joints_topic)
: Node("path_planner_node"),
  _move_group(move_group)
{
  _target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
    "arm_target_pose", 1,
    std::bind(&PathPlannerNode::receiveTargetPoseCallback, this, std::placeholders::_1));

  _arm_state_sub = this->create_subscription<std_msgs::msg::String>(
    "arm_state", 1,
    std::bind(&PathPlannerNode::updateStateCallback, this, std::placeholders::_1));

  _joint_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "arm_curr_angles", 1,
    std::bind(&PathPlannerNode::joint_callback, this, std::placeholders::_1));

  _joint_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "arm_ik_target_joints",
    1);

  _ik_success_pub = this->create_publisher<std_msgs::msg::Bool>(
    "ik_was_successful",
    1);
  // _rviz_joint_pose_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  _pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("arm_fk_pose", 1);
  _joint_path_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_path_joints", 1);
  _trajectory_pub = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("trajectory_joints", 1);

  _scene = std::make_shared<planning_scene::PlanningScene>(_move_group->getRobotModel());


  moveit::core::RobotModelConstPtr robot_model_ = _move_group->getRobotModel();
  jmg = robot_model_->getJointModelGroup(_move_group->getName());
  robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state->setToDefaultValues();
}

// ---------------------------------------------------------------------------

void PathPlannerNode::receiveTargetPoseCallback(
  const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const
{
  switch (_curr_state) {
    case ArmState::IK:           calculateIK(target_pose_msg);   break;
    case ArmState::PATH_PLANNING: calculatePath(target_pose_msg); break;
    default:
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning not configured for state");
      break;
  }
}

// ---------------------------------------------------------------------------

static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  T.linear() = q.toRotationMatrix();
  return T;
}

// ---------------------------------------------------------------------------

void PathPlannerNode::calculateIK(
  const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const
{
  // success message
  std_msgs::msg::Bool ik_success;
  ik_success.data = false;

  if (!_move_group) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized");
    _ik_success_pub->publish(ik_success);
    return;
  }

  moveit::core::RobotStatePtr current_state = robot_state;

  const moveit::core::JointModelGroup * jmg_ptr =
    current_state->getJointModelGroup(_move_group->getName());
  if (!jmg_ptr) {
    RCLCPP_ERROR(get_logger(), "JointModelGroup not found!");
    _ik_success_pub->publish(ik_success);
    return;
  }

  Eigen::Isometry3d target_pose_eigen = poseMsgToEigen(*target_pose_msg);

  // IK collision checks
  auto validity_cb =
  [this](moveit::core::RobotState * s,
         const moveit::core::JointModelGroup * g,
         const double * values) {
    s->setJointGroupPositions(g, values);
    s->update();
    collision_detection::CollisionRequest req;
    req.group_name = g->getName();
    collision_detection::CollisionResult res;
    _scene->checkSelfCollision(req, res, *s);
    return !res.collision;
  };

  bool found_ik = current_state->setFromIK(
    jmg_ptr,
    target_pose_eigen,
    _move_group->getEndEffectorLink(),
    0.1,
    validity_cb,
    kinematics::KinematicsQueryOptions());

  if (!found_ik) {
    RCLCPP_WARN(get_logger(), "IK solution not found for target pose");
    _ik_success_pub->publish(ik_success);
    return;
  }

  std::vector<double> joint_values;
  current_state->copyJointGroupPositions(jmg_ptr, joint_values);

  // joint_values layout: [j1, j2, j3, j4, j5, j6]
  //   indices              0   1   2   3   4   5
  if (joint_values.size() < NUM_JOINTS) {
    RCLCPP_ERROR(get_logger(), "Unexpected joint count from IK solver");
    _ik_success_pub->publish(ik_success);
    return;
  }

  // Reject branch flips / big jumps relative to the current (feedback) pose
  constexpr double kMaxJump = 0.15;  // rad, tune this
  const auto & names = jmg_ptr->getVariableNames();
  for (size_t i = 0; i < joint_values.size(); ++i) {
    const double d = joint_values[i] - robot_state->getVariablePosition(names[i]);
    if (std::abs(d) > kMaxJump) {
      RCLCPP_WARN(get_logger(),
        "IK jump on joint %zu: %.2f rad, rejecting", i + 1, d);
      return;
    }
  }

  // Need to swap joints 5 and 6 and invert 4-6
  double temp = joint_values[5];
  joint_values[5] = -joint_values[4];
  joint_values[4] = -temp;
  joint_values[3] = -joint_values[3];

  std_msgs::msg::Float32MultiArray msg;
  msg.data.assign(joint_values.begin(), joint_values.end());
  _joint_pose_pub->publish(msg);
  ik_success.data = true;
  _ik_success_pub->publish(ik_success);
}

// ---------------------------------------------------------------------------

void PathPlannerNode::joint_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::vector<double> joint_positions(msg->data.begin(), msg->data.end() - 1);

  // Need to swap joints 5 and 6 and invert 4-6
  double temp = joint_positions[5];
  joint_positions[5] = -joint_positions[4];
  joint_positions[4] = -temp;
  joint_positions[3] = -joint_positions[3];

  // Convert back to radians (lmao)
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_positions[i] = joint_positions[i] * 3.14 / 180.0;  // Convert degrees to radians
  }

  robot_state->setJointGroupPositions(jmg, joint_positions);
  robot_state->update();

  Eigen::Isometry3d tf = robot_state->getGlobalLinkTransform("gripper");

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
}

// ---------------------------------------------------------------------------

void PathPlannerNode::calculatePath(
  const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const
{
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

// ---------------------------------------------------------------------------

void PathPlannerNode::updateCurrPoseCallback(
  geometry_msgs::msg::Pose::SharedPtr curr_pose_msg)
{
  _curr_pose = curr_pose_msg;
}

// ---------------------------------------------------------------------------

void PathPlannerNode::updateStateCallback(
  std_msgs::msg::String::SharedPtr curr_state_msg)
{
  if (curr_state_msg->data == "IDLE") {
    _curr_state = ArmState::IDLE;
  } else if (curr_state_msg->data == "MANUAL") {
    _curr_state = ArmState::MANUAL;
  } else if (curr_state_msg->data == "IK") {
    _curr_state = ArmState::IK;
  } else if (curr_state_msg->data == "PATH_PLANNING") {
    _curr_state = ArmState::PATH_PLANNING;
  } else if (curr_state_msg->data == "SCIENCE") {_curr_state = ArmState::SCIENCE;}
}

// ---------------------------------------------------------------------------

void PathPlannerNode::publishPath(
  moveit_msgs::msg::RobotTrajectory & trajectory) const
{
  _trajectory_pub->publish(trajectory);
  // for (const auto& point : trajectory.joint_trajectory.points) {
  //     std::array<double, NUM_JOINTS> joint_positions{};
  //     for (size_t i = 0; i < NUM_JOINTS && i < point.positions.size(); ++i)
  //         joint_positions[i] = point.positions[i];

  //     std_msgs::msg::Float32MultiArray msg;
  //     msg.data.assign(joint_positions.begin(), joint_positions.end());
  //     _joint_path_pub->publish(msg);
  // }
}
