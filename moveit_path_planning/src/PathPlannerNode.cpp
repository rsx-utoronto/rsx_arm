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


namespace DiffWrist {

    static double wrap(double a)
    {
        return std::remainder(a, 2.0 * M_PI);
    }

    static Eigen::Matrix3d rotX(double a)
    {
        double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R << 1,  0,  0,
             0,  c, -s,
             0,  s,  c;
        return R;
    }

    static Eigen::Matrix3d rotY(double a)
    {
        double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R <<  c,  0,  s,
              0,  1,  0,
             -s,  0,  c;
        return R;
    }

    static Eigen::Matrix3d rotZ(double a)
    {
        double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R <<  c, -s,  0,
              s,  c,  0,
              0,  0,  1;
        return R;
    }

    // Decompose R = Rot_X(pitch) * Rot_Y(roll)
    //
    // Expanding:
    //   R = [ cosR        0      sinR     ]
    //       [ sinP sinR   cosP  -sinP cosR ]
    //       [-cosP sinR   sinP   cosP cosR ]
    //
    // roll  = atan2(  R[0,2],  R[0,0] )   (from row 0)
    // pitch = atan2(  R[2,1],  R[1,1] )   (from col 1)
    //
    // Gimbal lock when cos(roll) ≈ 0 → fix pitch = 0
    static std::pair<double,double> decomposeXY(const Eigen::Matrix3d& R)
    {
        double cos_roll = std::sqrt(R(0,0)*R(0,0) + R(0,2)*R(0,2));

        double pitch, roll;
        if (cos_roll < 1e-6) {      // gimbal lock: roll ≈ ±π/2
            pitch = 0.0;
            roll  = std::atan2(R(0,2), cos_roll);
        } else {
            roll  = std::atan2( R(0,2),  R(0,0));
            pitch = std::atan2(-R(2,1),  R(1,1));
        }
        return {pitch, roll};
    }

    struct Solution {
        double j4;            // elbow roll (radians)
        double j5;            // unused — kept for struct size compat, same as wrist_roll
        double j6;            // unused — kept for struct size compat, same as wrist_pitch
        double wrist_pitch;   // radians — publish as degrees at index 5
        double wrist_roll;    // radians — publish as degrees at index 4
    };

    // -----------------------------------------------------------------------
    // solve()
    //
    // R_pre_j4   : FK rotation from world/base through J1-J3 (the frame J4
    //              sits in).  J4 rotates about the LOCAL Z of this frame.
    // R_target   : desired EE rotation in the world/base frame.
    // j4_current : current elbow roll (radians) — biases solution toward
    //              minimal joint motion.
    // limits     : {j4_min, j4_max, j5_min, j5_max, j6_min, j6_max}
    //
    // Strategy
    // ---------
    // Express R_target in the J3-output (pre-J4) local frame:
    //   R_local = R_pre_j4^T * R_target
    //
    // R_local must be produced by the chain  Rot_Z(j4) * Rot_X(pitch) * Rot_Y(roll).
    //
    // The yaw component of R_local (rotation about Z) must come entirely from
    // J4 because the wrist has no yaw DOF.  Extract it analytically:
    //
    //   j4 = atan2( R_local[1,0], R_local[0,0] )   — the Z-axis Euler angle
    //
    // Then the wrist residual is:
    //   R_wrist = Rot_Z(-j4) * R_local
    //
    // which should now be expressible as Rot_X(pitch) * Rot_Y(roll).
    // Decompose and apply the differential inverse.
    //
    // Because this is analytical (not a search) it is exact and fast.
    // The only search remaining is over the ±2π wrap of j4 to find the
    // solution closest to j4_current within limits.
    // -----------------------------------------------------------------------
    static std::optional<Solution> solve(
        const Eigen::Matrix3d& R_pre_j4,
        const Eigen::Matrix3d& R_target,
        double j4_current,
        std::array<double,6> limits)
    {
        const double j4_min = limits[0], j4_max = limits[1];
        const double pitch_min = limits[2], pitch_max = limits[3];
        const double roll_min  = limits[4], roll_max  = limits[5];

        // --- 1. Express target in the J3-output local frame ---
        Eigen::Matrix3d R_local = R_pre_j4.transpose() * R_target;

        // --- 2. Extract the yaw component → this is J4 ---
        //   R_local = Rot_Z(j4) * R_wrist
        //   The Z-Euler angle of R_local is the yaw J4 must produce.
        double j4_raw = std::atan2(R_local(1, 0), R_local(0, 0));

        // --- 3. Wrist residual after J4 removes the yaw ---
        Eigen::Matrix3d R_wrist = rotZ(-j4_raw) * R_local;

        // --- 4. Decompose residual into Rot_X(pitch) * Rot_Y(roll) ---
        //   These are published directly in degrees to the CAN pipeline.
        //   calc_differential() in arm_can_utils.py converts them to
        //   motor angles:  motor1 = roll*WRIST_RATIO + pitch
        //                  motor2 = roll*WRIST_RATIO - pitch
        auto [pitch, roll] = decomposeXY(R_wrist);

        // --- 5. Find the best ±2π wrap of j4 closest to j4_current ---
        std::optional<Solution> best;
        double best_cost = std::numeric_limits<double>::infinity();

        for (int k = -1; k <= 1; ++k) {
            double j4 = j4_raw + k * 2.0 * M_PI;

            if (j4    < j4_min    || j4    > j4_max)    continue;
            if (pitch < pitch_min || pitch > pitch_max) continue;
            if (roll  < roll_min  || roll  > roll_max)  continue;

            double cost = (j4 - j4_current) * (j4 - j4_current);
            if (cost < best_cost) {
                best_cost = cost;
                best = Solution{j4, 0.0, 0.0, pitch, roll};
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

    // Joint limits for J4 only — J5/J6 limits are not needed here because
    // we publish roll/pitch directly and calc_differential() handles the
    // motor coupling on the CAN side.
    std::array<double,6> limits = {
        -M_PI,  M_PI,   // J4 elbow roll
        -M_PI,  M_PI,   // wrist pitch (±180°)
        -M_PI,  M_PI    // wrist roll  (±180°)
    };

    // ------------------------------------------------------------------
    // Step 4 — Differential wrist solver
    // ------------------------------------------------------------------
    // Note: generate_data_packet() in arm_can_utils.py calls calc_differential()
    // which handles the motor coupling:
    //   motor1 = roll * WRIST_RATIO + pitch
    //   motor2 = roll * WRIST_RATIO - pitch
    //
    // It expects data_list[-3] = roll (degrees), data_list[-2] = pitch (degrees).
    // So we publish wrist_roll and wrist_pitch in degrees directly — do NOT
    // pre-compute motor angles here, the CAN layer does that.
    // ------------------------------------------------------------------
    auto wrist_solution = DiffWrist::solve(
        R_pre_j4, R_target, j4_current, limits);

    if (!wrist_solution) {
        RCLCPP_WARN(get_logger(),
            "Differential wrist solver found no feasible solution — "
            "publishing position-only IK result");
    } else {
        // J4: elbow roll — stays in radians (your arm node handles conversion)
        joint_values[3] = wrist_solution->j4;

        // J5, J6: publish roll and pitch in degrees so calc_differential()
        // receives the values it expects.
        // data_list[-3] = roll  → index 4
        // data_list[-2] = pitch → index 5
        joint_values[4] = wrist_solution->wrist_roll  * (180.0 / M_PI);  // roll  → degrees
        joint_values[5] = wrist_solution->wrist_pitch * (180.0 / M_PI);  // pitch → degrees

        RCLCPP_DEBUG(get_logger(),
            "Wrist solved — J4=%.3frad  roll=%.2f°  pitch=%.2f°",
            wrist_solution->j4,
            joint_values[4],
            joint_values[5]);
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
    if      (curr_state_msg->data == "IDLE")          _curr_state = ArmState::IDLE;
    else if (curr_state_msg->data == "MANUAL")        _curr_state = ArmState::MANUAL;
    else if (curr_state_msg->data == "IK")            _curr_state = ArmState::IK;
    else if (curr_state_msg->data == "PATH_PLANNING") _curr_state = ArmState::PATH_PLANNING;
    else if (curr_state_msg->data == "SCIENCE")       _curr_state = ArmState::SCIENCE;
}

// ---------------------------------------------------------------------------

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