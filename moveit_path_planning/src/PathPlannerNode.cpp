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
// Differential wrist solver (ported from diff_wrist_solver.py)
// ---------------------------------------------------------------------------

namespace DiffWrist {

    // Rotation matrix about Z axis
    static Eigen::Matrix3d rotZ(double angle)
    {
        Eigen::Matrix3d R;
        double c = std::cos(angle), s = std::sin(angle);
        R <<  c, -s,  0,
              s,  c,  0,
              0,  0,  1;
        return R;
    }

    // Rotation matrix about Y axis
    static Eigen::Matrix3d rotY(double angle)
    {
        Eigen::Matrix3d R;
        double c = std::cos(angle), s = std::sin(angle);
        R <<  c,  0,  s,
              0,  1,  0,
             -s,  0,  c;
        return R;
    }

    // Wrap angle to (-π, π]
    static double wrap(double a)
    {
        return std::remainder(a, 2.0 * M_PI);
    }

    // Decompose R = Rot_Z(alpha) * Rot_Y(beta) → returns {alpha, beta}
    //
    // R = [cosα cosβ   -sinα   cosα sinβ]
    //     [sinα cosβ    cosα   sinα sinβ]
    //     [  -sinβ        0      cosβ   ]
    //
    // alpha = atan2( R[1,0], R[0,0] )
    // beta  = atan2(-R[2,0], R[2,2] )
    //
    // Gimbal lock guard: when cos(beta) ≈ 0 fix alpha = 0
    static std::pair<double,double> decomposeZY(const Eigen::Matrix3d& R)
    {
        double cos_beta = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));

        double alpha, beta;
        if (cos_beta < 1e-6) {          // near gimbal lock
            alpha = 0.0;
            beta  = std::atan2(-R(2,0), cos_beta);
        } else {
            alpha = std::atan2(R(1,0),  R(0,0));
            beta  = std::atan2(-R(2,0), R(2,2));
        }
        return {alpha, beta};
    }

    struct Solution {
        double j4, j5, j6;
        double wrist_roll, wrist_pitch;
    };

    // Main solver — mirrors solve_diff_wrist() in the Python file.
    //
    // R_pre_j4   : FK rotation from base through J1-J3 (Eigen 3x3)
    // R_target   : desired EE rotation (Eigen 3x3)
    // j4_current : current elbow roll (radians), used to bias solution
    // limits     : {j4_min, j4_max, j5_min, j5_max, j6_min, j6_max}
    // j4_samples : how many elbow-roll candidates to try (default 64)
    static std::optional<Solution> solve(
        const Eigen::Matrix3d& R_pre_j4,
        const Eigen::Matrix3d& R_target,
        double j4_current,
        std::array<double,6> limits,   // {j4min,j4max, j5min,j5max, j6min,j6max}
        int j4_samples = 64)
    {
        const double j4_min = limits[0], j4_max = limits[1];
        const double j5_min = limits[2], j5_max = limits[3];
        const double j6_min = limits[4], j6_max = limits[5];

        std::optional<Solution> best;
        double best_cost = std::numeric_limits<double>::infinity();

        for (int i = 0; i < j4_samples; ++i)
        {
            double j4 = j4_min + (j4_max - j4_min) * i / (j4_samples - 1);

            // --- 1. Rotation through J4 ---
            Eigen::Matrix3d R_j4        = rotZ(j4);
            Eigen::Matrix3d R_through_j4 = R_pre_j4 * R_j4;

            // --- 2. Residual rotation the wrist must produce ---
            //    R_through_j4 * R_wrist = R_target
            //    R_wrist = R_through_j4^T * R_target
            Eigen::Matrix3d R_wrist = R_through_j4.transpose() * R_target;

            // --- 3. ZY decomposition → wrist roll α, pitch β ---
            auto [alpha, beta] = decomposeZY(R_wrist);

            // Needs further debugging, seems like our axes are a bit off but i can't tell for sure
            beta = -beta;
            alpha = -alpha;

            // --- 4. Differential inverse ---
            //    J5 = α + β
            //    J6 = α - β
            double j5 = wrap(alpha + beta);
            double j6 = wrap(alpha - beta);

            // --- 5. Feasibility ---
            if (j4 < j4_min || j4 > j4_max) continue;
            if (j5 < j5_min || j5 > j5_max) continue;
            if (j6 < j6_min || j6 > j6_max) continue;

            // --- 6. Cost: minimise motion from current J4 ---
            double cost = (j4 - j4_current) * (j4 - j4_current);
            if (cost < best_cost) {
                best_cost = cost;
                best = Solution{j4, j5, j6, alpha, beta};
            }
        }

        return best;
    }

} // namespace DiffWrist


// ---------------------------------------------------------------------------
// PathPlannerNode
// ---------------------------------------------------------------------------

PathPlannerNode::PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group,
                                 const std::string& target_pose_topic,
                                 const std::string& target_joints_topic)
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

    _joint_pose_pub  = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_ik_target_joints", 1);
    _rviz_joint_pose_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    _pose_pub        = this->create_publisher<geometry_msgs::msg::Pose>("arm_fk_pose", 1);
    _joint_path_pub  = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_path_joints", 1);

    moveit::core::RobotModelConstPtr robot_model_ = _move_group->getRobotModel();
    jmg          = robot_model_->getJointModelGroup(_move_group->getName());
    robot_state  = std::make_shared<moveit::core::RobotState>(robot_model_);
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

static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose)
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
    if (!_move_group) {
        RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized");
        return;
    }

    moveit::core::RobotStatePtr current_state = robot_state;

    const moveit::core::JointModelGroup* jmg_ptr =
        current_state->getJointModelGroup(_move_group->getName());
    if (!jmg_ptr) {
        RCLCPP_ERROR(get_logger(), "JointModelGroup not found!");
        return;
    }

    Eigen::Isometry3d target_pose_eigen = poseMsgToEigen(*target_pose_msg);

    // ------------------------------------------------------------------
    // Step 1 — TRAC-IK solves position + gives an approximate orientation
    //           for the first (NUM_JOINTS - 3) joints (J1-J3).
    //           We pass the full target so it converges on position; the
    //           orientation for J4-J6 will be overwritten below.
    // ------------------------------------------------------------------
    bool found_ik = current_state->setFromIK(
        jmg_ptr,
        target_pose_eigen,
        _move_group->getEndEffectorLink(),
        0.1,
        moveit::core::GroupStateValidityCallbackFn(),
        kinematics::KinematicsQueryOptions());

    if (!found_ik) {
        RCLCPP_WARN(get_logger(), "IK solution not found for target pose");
        return;
    }

    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(jmg_ptr, joint_values);

    // joint_values layout: [j1, j2, j3, j4, j5, j6]
    //   indices              0   1   2   3   4   5
    if (joint_values.size() < NUM_JOINTS) {
        RCLCPP_ERROR(get_logger(), "Unexpected joint count from IK solver");
        return;
    }

    // ------------------------------------------------------------------
    // Step 2 — Build FK rotation through J1-J3 using the IK result.
    //
    //   We read the transform of the link immediately before J4
    //   (the "elbow roll input" frame) from MoveIt's robot state.
    //   This is the same frame used in fk_to_elbow.py.
    //
    //   Adjust "link_3" to whatever your URDF calls the link whose
    //   child joint is J4 (elbow roll).
    // ------------------------------------------------------------------
    current_state->update();
    const Eigen::Isometry3d T_pre_j4 =
        current_state->getGlobalLinkTransform("link_3");   // ← adjust if needed
    const Eigen::Matrix3d R_pre_j4 = T_pre_j4.rotation();

    // ------------------------------------------------------------------
    // Step 3 — Desired EE orientation from the target pose message
    // ------------------------------------------------------------------
    Eigen::Quaterniond q_tgt(
        target_pose_msg->orientation.w,
        target_pose_msg->orientation.x,
        target_pose_msg->orientation.y,
        target_pose_msg->orientation.z);
    const Eigen::Matrix3d R_target = q_tgt.toRotationMatrix();

    // Current J4 reading (used to bias the solution toward minimal motion)
    double j4_current = joint_values[3];

    // Joint limits — adjust to match your URDF / hardware
    std::array<double,6> limits = {
        -M_PI,      M_PI,        // J4 elbow roll
        -2*M_PI,    2*M_PI,      // J5 motor 1
        -2*M_PI,    2*M_PI       // J6 motor 2
    };

    // ------------------------------------------------------------------
    // Step 4 — Differential wrist solver
    // ------------------------------------------------------------------
    auto wrist_solution = DiffWrist::solve(
        R_pre_j4, R_target, j4_current, limits);

    if (!wrist_solution) {
        RCLCPP_WARN(get_logger(),
            "Differential wrist solver found no feasible solution — "
            "publishing position-only IK result");
        // Fall through and publish whatever TRAC-IK gave us
    } else {
        // Overwrite J4, J5, J6 with the analytically correct values
        joint_values[3] = wrist_solution->j4;
        joint_values[4] = wrist_solution->j5;
        joint_values[5] = wrist_solution->j6;

        RCLCPP_DEBUG(get_logger(),
            "Wrist solved — J4=%.3f J5=%.3f J6=%.3f  "
            "(roll=%.3f  pitch=%.3f)",
            wrist_solution->j4,
            wrist_solution->j5,
            wrist_solution->j6,
            wrist_solution->wrist_roll,
            wrist_solution->wrist_pitch);
    }

    // ------------------------------------------------------------------
    // Step 5 — Publish
    // ------------------------------------------------------------------  
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(joint_values.begin(), joint_values.end());
    _joint_pose_pub->publish(msg);
}

// ---------------------------------------------------------------------------

void PathPlannerNode::joint_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::vector<double> joint_positions(msg->data.begin(), msg->data.end());

    robot_state->setJointGroupPositions(jmg, joint_positions);
    robot_state->update();

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
}

void PathPlannerNode::calculatePath(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const
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

void PathPlannerNode::updateCurrPoseCallback(
    geometry_msgs::msg::Pose::SharedPtr curr_pose_msg)
{
    _curr_pose = curr_pose_msg;
}

void PathPlannerNode::updateStateCallback(
    std_msgs::msg::String::SharedPtr curr_state_msg)
{
    if      (curr_state_msg->data == "IDLE")          _curr_state = ArmState::IDLE;
    else if (curr_state_msg->data == "MANUAL")        _curr_state = ArmState::MANUAL;
    else if (curr_state_msg->data == "IK")            _curr_state = ArmState::IK;
    else if (curr_state_msg->data == "PATH_PLANNING") _curr_state = ArmState::PATH_PLANNING;
    else if (curr_state_msg->data == "SCIENCE")       _curr_state = ArmState::SCIENCE;
}

void PathPlannerNode::publishPath(
    moveit_msgs::msg::RobotTrajectory& trajectory) const
{
    for (const auto& point : trajectory.joint_trajectory.points) {
        std::array<double, NUM_JOINTS> joint_positions{};
        for (size_t i = 0; i < NUM_JOINTS && i < point.positions.size(); ++i)
            joint_positions[i] = point.positions[i];

        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(joint_positions.begin(), joint_positions.end());
        _joint_path_pub->publish(msg);
    }
}