#ifndef MOVEIT_PATH_PLANNING__PLANNER_SERVER_HPP_
#define MOVEIT_PATH_PLANNING__PLANNER_SERVER_HPP_

#include <memory>
#include <string>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "arm_msgs/srv/plan_motion.hpp"
#include "arm_msgs/action/plan_and_execute.hpp"

namespace moveit_path_planning
{

class PlannerServer : public rclcpp::Node
{
public:
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlannerServer() = default;

private:
  // Service and action types
  using PlanMotion = arm_msgs::srv::PlanMotion;
  using PlanAndExecute = arm_msgs::action::PlanAndExecute;
  using GoalHandlePlanAndExecute = rclcpp_action::ServerGoalHandle<PlanAndExecute>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  // Core MoveIt components
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  
  // ROS interfaces
  rclcpp::Service<PlanMotion>::SharedPtr plan_service_;
  rclcpp_action::Server<PlanAndExecute>::SharedPtr plan_execute_action_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr execute_client_;

  // Parameters
  std::string planning_group_;
  std::string base_frame_;
  std::string ee_frame_;
  std::string default_planner_id_;
  double default_planning_time_;
  int default_num_attempts_;
  double default_velocity_scaling_;
  double default_acceleration_scaling_;
  double default_cartesian_step_;
  double default_cartesian_jump_threshold_;
  bool default_allow_replanning_;
  bool use_time_optimal_parameterization_;
  
  // Random number generator for deterministic mode
  std::mt19937_64 rng_;

  // Service callbacks
  void handlePlanMotion(
    const std::shared_ptr<PlanMotion::Request> request,
    std::shared_ptr<PlanMotion::Response> response);

  // Action callbacks
  rclcpp_action::GoalResponse handlePlanAndExecuteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanAndExecute::Goal> goal);
  
  rclcpp_action::CancelResponse handlePlanAndExecuteCancel(
    const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle);
  
  void handlePlanAndExecuteAccepted(
    const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle);

  void executePlanAndExecute(
    const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle);

  // Core planning methods
  bool planToJointTarget(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_values,
    const std::string & planner_id,
    double planning_time,
    int num_attempts,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    std::string & error_msg);

  bool planToPoseTarget(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & planner_id,
    double planning_time,
    int num_attempts,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    std::string & error_msg);

  bool planCartesianPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    double max_step,
    double jump_threshold,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    double & fraction_achieved,
    std::string & error_msg);

  // Trajectory processing
  bool timeParameterizeTrajectory(
    robot_trajectory::RobotTrajectory & trajectory,
    double velocity_scaling,
    double acceleration_scaling);

  bool validateTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & error_msg);

  // Execution
  bool executeTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & error_msg);

  // Utilities
  void applyConstraints(
    const moveit_msgs::msg::Constraints & constraints);

  void setupDeterministicPlanning(uint64_t seed);

  void declareParameters();
  void loadParameters();
};

}  // namespace moveit_path_planning

#endif  // MOVEIT_PATH_PLANNING__PLANNER_SERVER_HPP_
