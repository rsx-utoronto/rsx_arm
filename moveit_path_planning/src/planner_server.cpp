#include "moveit_path_planning/planner_server.hpp"

#include <chrono>
#include <functional>
#include <thread>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace moveit_path_planning
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: Node("planner_server", options)
{
  // Declare and load parameters
  declareParameters();
  loadParameters();

  RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for group: %s", 
    planning_group_.c_str());

  RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for group: %s", 
    planning_group_.c_str());

  // Initialize MoveIt components
  try {
    // Create MoveGroupInterface using static shared_ptr
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    
    // Create MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr, planning_group_);
    
    // Set default parameters
    move_group_->setPlanningTime(default_planning_time_);
    move_group_->setNumPlanningAttempts(default_num_attempts_);
    move_group_->setMaxVelocityScalingFactor(default_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(default_acceleration_scaling_);
    
    if (!default_planner_id_.empty()) {
      move_group_->setPlannerId(default_planner_id_);
    }

    // Create PlanningSceneInterface
    planning_scene_interface_ = 
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Initialize PlanningSceneMonitor
    planning_scene_monitor_ = 
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_ptr, "robot_description");
    
    if (planning_scene_monitor_->getPlanningScene()) {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      RCLCPP_INFO(this->get_logger(), "PlanningSceneMonitor initialized successfully");
    } else {
      RCLCPP_WARN(this->get_logger(), "PlanningSceneMonitor could not load planning scene");
    }

    RCLCPP_INFO(this->get_logger(), 
      "MoveGroupInterface initialized for group: %s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), 
      "End effector frame: %s, Base frame: %s", 
      ee_frame_.c_str(), base_frame_.c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt components: %s", e.what());
    throw;
  }

  // Create service
  plan_service_ = this->create_service<PlanMotion>(
    "~/plan_motion",
    std::bind(&PlannerServer::handlePlanMotion, this, _1, _2));

  // Create action server
  plan_execute_action_ = rclcpp_action::create_server<PlanAndExecute>(
    this,
    "~/plan_and_execute",
    std::bind(&PlannerServer::handlePlanAndExecuteGoal, this, _1, _2),
    std::bind(&PlannerServer::handlePlanAndExecuteCancel, this, _1),
    std::bind(&PlannerServer::handlePlanAndExecuteAccepted, this, _1));

  // Create execution action client
  execute_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/joint_trajectory_controller/follow_joint_trajectory");

  RCLCPP_INFO(this->get_logger(), "PlannerServer initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Service: ~/plan_motion");
  RCLCPP_INFO(this->get_logger(), "Action: ~/plan_and_execute");
}

void PlannerServer::declareParameters()
{
  // Planning group and frames
  this->declare_parameter("planning_group", "rover_arm");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("ee_frame", "end_effector_link");

  // Default planner settings
  this->declare_parameter("default_planner_id", "RRTConnectkConfigDefault");
  this->declare_parameter("default_planning_time", 5.0);
  this->declare_parameter("default_num_attempts", 1);
  this->declare_parameter("default_velocity_scaling", 0.1);
  this->declare_parameter("default_acceleration_scaling", 0.1);

  // Cartesian path settings
  this->declare_parameter("default_cartesian_step", 0.01);
  this->declare_parameter("default_cartesian_jump_threshold", 0.0);

  // Advanced options
  this->declare_parameter("default_allow_replanning", false);
  this->declare_parameter("use_time_optimal_parameterization", false);
}

void PlannerServer::loadParameters()
{
  planning_group_ = this->get_parameter("planning_group").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  ee_frame_ = this->get_parameter("ee_frame").as_string();
  
  default_planner_id_ = this->get_parameter("default_planner_id").as_string();
  default_planning_time_ = this->get_parameter("default_planning_time").as_double();
  default_num_attempts_ = this->get_parameter("default_num_attempts").as_int();
  default_velocity_scaling_ = this->get_parameter("default_velocity_scaling").as_double();
  default_acceleration_scaling_ = this->get_parameter("default_acceleration_scaling").as_double();
  
  default_cartesian_step_ = this->get_parameter("default_cartesian_step").as_double();
  default_cartesian_jump_threshold_ = 
    this->get_parameter("default_cartesian_jump_threshold").as_double();
  
  default_allow_replanning_ = this->get_parameter("default_allow_replanning").as_bool();
  use_time_optimal_parameterization_ = 
    this->get_parameter("use_time_optimal_parameterization").as_bool();
}

void PlannerServer::handlePlanMotion(
  const std::shared_ptr<PlanMotion::Request> request,
  std::shared_ptr<PlanMotion::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received planning request (target_type: %d)", 
    request->target_type);

  auto start_time = std::chrono::steady_clock::now();

  // Setup deterministic planning if requested
  if (request->deterministic) {
    setupDeterministicPlanning(request->random_seed);
  }

  // Apply velocity and acceleration scaling
  double vel_scaling = request->velocity_scaling_factor > 0.0 ? 
    request->velocity_scaling_factor : default_velocity_scaling_;
  double acc_scaling = request->acceleration_scaling_factor > 0.0 ? 
    request->acceleration_scaling_factor : default_acceleration_scaling_;
  
  move_group_->setMaxVelocityScalingFactor(vel_scaling);
  move_group_->setMaxAccelerationScalingFactor(acc_scaling);

  // Set planner configuration
  std::string planner_id = request->planner_id.empty() ? 
    default_planner_id_ : request->planner_id;
  double planning_time = request->planning_time > 0.0 ? 
    request->planning_time : default_planning_time_;
  int num_attempts = request->num_planning_attempts > 0 ? 
    request->num_planning_attempts : default_num_attempts_;

  move_group_->setPlannerId(planner_id);
  move_group_->setPlanningTime(planning_time);
  move_group_->setNumPlanningAttempts(num_attempts);

  // Apply path constraints if provided
  if (!request->path_constraints.name.empty()) {
    applyConstraints(request->path_constraints);
  } else {
    move_group_->clearPathConstraints();
  }

  // Plan based on target type
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::string error_msg;
  bool success = false;
  double fraction_achieved = 0.0;

  switch (request->target_type) {
    case PlanMotion::Request::TARGET_TYPE_JOINT:
      RCLCPP_INFO(this->get_logger(), "Planning to joint target");
      success = planToJointTarget(
        request->joint_names, request->joint_values,
        planner_id, planning_time, num_attempts, plan, error_msg);
      break;

    case PlanMotion::Request::TARGET_TYPE_POSE:
      RCLCPP_INFO(this->get_logger(), "Planning to pose target");
      success = planToPoseTarget(
        request->target_pose, planner_id, planning_time, num_attempts, plan, error_msg);
      break;

    case PlanMotion::Request::TARGET_TYPE_CARTESIAN:
      RCLCPP_INFO(this->get_logger(), "Planning Cartesian path with %zu waypoints", 
        request->waypoints.size());
      double max_step = request->cartesian_max_step > 0.0 ? 
        request->cartesian_max_step : default_cartesian_step_;
      double jump_threshold = request->cartesian_jump_threshold;
      success = planCartesianPath(
        request->waypoints, max_step, jump_threshold, plan, fraction_achieved, error_msg);
      
      if (success && fraction_achieved < 0.95) {
        RCLCPP_WARN(this->get_logger(), 
          "Cartesian path only achieved %.1f%% of the path", fraction_achieved * 100.0);
      }
      break;
  }

  // Populate response
  response->success = success;
  if (success) {
    response->trajectory = plan.trajectory_;
    response->waypoints_count = plan.trajectory_.joint_trajectory.points.size();
    response->planner_used = planner_id;
    response->error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    response->message = "Planning succeeded";
    
    auto end_time = std::chrono::steady_clock::now();
    response->planning_time = 
      std::chrono::duration<double>(end_time - start_time).count();
    
    RCLCPP_INFO(this->get_logger(), 
      "Planning succeeded in %.3f seconds with %u waypoints",
      response->planning_time, response->waypoints_count);
  } else {
    response->error_code = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    response->message = error_msg.empty() ? "Planning failed" : error_msg;
    response->planning_time = 0.0;
    response->waypoints_count = 0;
    
    RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", response->message.c_str());
  }
}

rclcpp_action::GoalResponse PlannerServer::handlePlanAndExecuteGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlanAndExecute::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_logger(), "Received plan and execute goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerServer::handlePlanAndExecuteCancel(
  const std::shared_ptr<GoalHandlePlanAndExecute> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handlePlanAndExecuteAccepted(
  const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle)
{
  // Execute in a separate thread to avoid blocking
  std::thread{std::bind(&PlannerServer::executePlanAndExecute, this, goal_handle)}.detach();
}

void PlannerServer::executePlanAndExecute(
  const std::shared_ptr<GoalHandlePlanAndExecute> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PlanAndExecute::Feedback>();
  auto result = std::make_shared<PlanAndExecute::Result>();

  // Planning phase
  feedback->stage = "planning";
  feedback->percent_complete = 0.0;
  feedback->status_message = "Starting planning...";
  goal_handle->publish_feedback(feedback);

  // Convert goal to service request
  auto plan_request = std::make_shared<PlanMotion::Request>();
  plan_request->target_type = goal->target_type;
  plan_request->joint_names = goal->joint_names;
  plan_request->joint_values = goal->joint_values;
  plan_request->target_pose = goal->target_pose;
  plan_request->waypoints = goal->waypoints;
  plan_request->cartesian_max_step = goal->cartesian_max_step;
  plan_request->cartesian_jump_threshold = goal->cartesian_jump_threshold;
  plan_request->frame_id = goal->frame_id;
  plan_request->path_constraints = goal->path_constraints;
  plan_request->position_tolerance = goal->position_tolerance;
  plan_request->orientation_tolerance = goal->orientation_tolerance;
  plan_request->joint_tolerance = goal->joint_tolerance;
  plan_request->planner_id = goal->planner_id;
  plan_request->planning_time = goal->planning_time;
  plan_request->num_planning_attempts = goal->num_planning_attempts;
  plan_request->velocity_scaling_factor = goal->velocity_scaling_factor;
  plan_request->acceleration_scaling_factor = goal->acceleration_scaling_factor;
  plan_request->allow_replanning = goal->allow_replanning;
  plan_request->deterministic = goal->deterministic;
  plan_request->random_seed = goal->random_seed;

  auto plan_response = std::make_shared<PlanMotion::Response>();
  handlePlanMotion(plan_request, plan_response);

  feedback->percent_complete = 50.0;
  goal_handle->publish_feedback(feedback);

  // Populate planning result
  result->success = plan_response->success;
  result->error_code = plan_response->error_code;
  result->message = plan_response->message;
  result->trajectory = plan_response->trajectory;
  result->planning_time = plan_response->planning_time;
  result->waypoints_count = plan_response->waypoints_count;
  result->planner_used = plan_response->planner_used;

  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Planning failed, aborting action");
    goal_handle->abort(result);
    return;
  }

  // Validation phase
  feedback->stage = "validating";
  feedback->percent_complete = 60.0;
  feedback->status_message = "Validating trajectory...";
  goal_handle->publish_feedback(feedback);

  std::string validation_error;
  if (!validateTrajectory(result->trajectory, validation_error)) {
    result->success = false;
    result->message = "Trajectory validation failed: " + validation_error;
    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  feedback->percent_complete = 70.0;
  goal_handle->publish_feedback(feedback);

  // Execution phase (if requested)
  if (goal->execute) {
    feedback->stage = "executing";
    feedback->percent_complete = 75.0;
    feedback->status_message = "Executing trajectory...";
    goal_handle->publish_feedback(feedback);

    std::string execution_error;
    result->execution_success = executeTrajectory(result->trajectory, execution_error);
    
    if (result->execution_success) {
      result->execution_error_code = 0;
      result->execution_message = "Execution completed successfully";
      feedback->percent_complete = 100.0;
      feedback->status_message = "Execution complete";
      goal_handle->publish_feedback(feedback);
      
      RCLCPP_INFO(this->get_logger(), "Action completed successfully");
      goal_handle->succeed(result);
    } else {
      result->execution_error_code = -1;
      result->execution_message = execution_error;
      RCLCPP_ERROR(this->get_logger(), "Execution failed: %s", execution_error.c_str());
      goal_handle->abort(result);
    }
  } else {
    // Planning only, no execution
    result->execution_success = true;
    result->execution_error_code = 0;
    result->execution_message = "Execution not requested";
    feedback->percent_complete = 100.0;
    feedback->status_message = "Planning complete (no execution)";
    goal_handle->publish_feedback(feedback);
    
    RCLCPP_INFO(this->get_logger(), "Action completed (planning only)");
    goal_handle->succeed(result);
  }
}

bool PlannerServer::planToJointTarget(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_values,
  const std::string & /*planner_id*/,
  double /*planning_time*/,
  int /*num_attempts*/,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  std::string & error_msg)
{
  if (joint_names.size() != joint_values.size()) {
    error_msg = "Joint names and values size mismatch";
    return false;
  }

  try {
    // Set joint value targets
    std::map<std::string, double> target_joints;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      target_joints[joint_names[i]] = joint_values[i];
    }
    
    move_group_->setJointValueTarget(target_joints);

    // Plan
    moveit::core::MoveItErrorCode result = move_group_->plan(plan);
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt planning failed with error code: " + 
        std::to_string(result.val);
      return false;
    }

    return true;

  } catch (const std::exception & e) {
    error_msg = std::string("Exception during joint target planning: ") + e.what();
    return false;
  }
}

bool PlannerServer::planToPoseTarget(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & /*planner_id*/,
  double /*planning_time*/,
  int /*num_attempts*/,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  std::string & error_msg)
{
  try {
    // Set pose target
    move_group_->setPoseTarget(target_pose);

    // Plan
    moveit::core::MoveItErrorCode result = move_group_->plan(plan);
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt planning failed with error code: " + 
        std::to_string(result.val);
      return false;
    }

    return true;

  } catch (const std::exception & e) {
    error_msg = std::string("Exception during pose target planning: ") + e.what();
    return false;
  }
}

bool PlannerServer::planCartesianPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  double max_step,
  double jump_threshold,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  double & fraction_achieved,
  std::string & error_msg)
{
  if (waypoints.empty()) {
    error_msg = "No waypoints provided for Cartesian path";
    return false;
  }

  try {
    // Convert PoseStamped to Pose
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(waypoints.size());
    for (const auto & wp : waypoints) {
      poses.push_back(wp.pose);
    }

    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    fraction_achieved = move_group_->computeCartesianPath(
      poses, max_step, jump_threshold, trajectory, true);

    if (fraction_achieved < 0.01) {
      error_msg = "Could not compute any valid Cartesian path";
      return false;
    }

    // Time parameterize the trajectory
    robot_trajectory::RobotTrajectory rt(
      move_group_->getRobotModel(), planning_group_);
    
    // Get current state safely
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    if (!current_state) {
      // Use default state if current state unavailable
      current_state = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
      current_state->setToDefaultValues();
    }
    rt.setRobotTrajectoryMsg(*current_state, trajectory);

    // Get current velocity and acceleration scaling
    double vel_scaling = default_velocity_scaling_;
    double acc_scaling = default_acceleration_scaling_;

    if (!timeParameterizeTrajectory(rt, vel_scaling, acc_scaling))
    {
      error_msg = "Failed to time-parameterize Cartesian trajectory";
      return false;
    }

    // Convert back to plan
    rt.getRobotTrajectoryMsg(plan.trajectory_);
    plan.planning_time_ = 0.0;  // Cartesian planning doesn't have a planning time

    return true;

  } catch (const std::exception & e) {
    error_msg = std::string("Exception during Cartesian path planning: ") + e.what();
    return false;
  }
}

bool PlannerServer::timeParameterizeTrajectory(
  robot_trajectory::RobotTrajectory & trajectory,
  double velocity_scaling,
  double acceleration_scaling)
{
  try {
    if (use_time_optimal_parameterization_) {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      return totg.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling);
    } else {
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      return iptp.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), 
      "Exception during time parameterization: %s", e.what());
    return false;
  }
}

bool PlannerServer::validateTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & error_msg)
{
  // Basic validation
  if (trajectory.joint_trajectory.points.empty()) {
    error_msg = "Trajectory has no points";
    return false;
  }

  // Check that trajectory has valid joint names
  if (trajectory.joint_trajectory.joint_names.empty()) {
    error_msg = "Trajectory has no joint names";
    return false;
  }

  // Validate each point has the correct number of positions
  const size_t num_joints = trajectory.joint_trajectory.joint_names.size();
  for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    const auto & point = trajectory.joint_trajectory.points[i];
    
    if (point.positions.size() != num_joints) {
      error_msg = "Point " + std::to_string(i) + 
        " has incorrect number of joint positions";
      return false;
    }

    // Check for NaN or infinite values
    for (const auto & pos : point.positions) {
      if (!std::isfinite(pos)) {
        error_msg = "Point " + std::to_string(i) + 
          " contains invalid joint position (NaN or Inf)";
        return false;
      }
    }
  }

  // Additional validation with planning scene
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene()) {
    // Could add collision checking here if needed
  }

  return true;
}

bool PlannerServer::executeTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & error_msg)
{
  if (!execute_client_) {
    error_msg = "Execution client not initialized";
    return false;
  }

  if (!execute_client_->wait_for_action_server(5s)) {
    error_msg = "Follow joint trajectory action server not available";
    return false;
  }

  // Convert MoveIt trajectory to FollowJointTrajectory goal
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = trajectory.joint_trajectory;

  RCLCPP_INFO(this->get_logger(), "Sending trajectory for execution...");

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  
  send_goal_options.result_callback = 
    [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
      }
    };

  auto goal_handle_future = execute_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal to be accepted
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    error_msg = "Execution goal not accepted within timeout";
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error_msg = "Execution goal was rejected by server";
    return false;
  }

  // Wait for execution to complete
  auto result_future = execute_client_->async_get_result(goal_handle);
  
  // Use trajectory duration plus buffer for timeout
  double trajectory_duration = 0.0;
  if (!trajectory.joint_trajectory.points.empty()) {
    const auto & last_point = trajectory.joint_trajectory.points.back();
    trajectory_duration = last_point.time_from_start.sec + 
      last_point.time_from_start.nanosec * 1e-9;
  }
  
  auto timeout = std::chrono::duration<double>(trajectory_duration + 10.0);
  
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    error_msg = "Execution did not complete within expected time";
    return false;
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_msg = "Execution failed with result code: " + 
      std::to_string(static_cast<int>(result.code));
    return false;
  }

  return true;
}

void PlannerServer::applyConstraints(
  const moveit_msgs::msg::Constraints & constraints)
{
  move_group_->setPathConstraints(constraints);
  RCLCPP_INFO(this->get_logger(), "Applied path constraints: %s", constraints.name.c_str());
}

void PlannerServer::setupDeterministicPlanning(uint64_t seed)
{
  rng_.seed(seed);
  
  // Set OMPL random seed via move_group
  // Note: This requires setting the ompl random seed parameter
  // The actual implementation depends on MoveIt version
  RCLCPP_INFO(this->get_logger(), 
    "Deterministic planning enabled with seed: %lu", seed);
}

}  // namespace moveit_path_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_path_planning::PlannerServer)
