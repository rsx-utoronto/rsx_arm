#include "../include/PathPlannerNode.hpp"
#include <chrono>

using namespace std::chrono_literals;

PathPlannerNode::PathPlannerNode(moveit::planning_interface::MoveGroupInterface* move_group, const std::string& target_pose_topic, const std::string& target_joints_topic):
Node("path_planner_node"),
_move_group(move_group)
{
    _target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("arm_target_pose", 1,
                            std::bind(&PathPlannerNode::receiveTargetPoseCallback, this, std::placeholders::_1));
    _curr_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("arm_curr_pose", 1,
                            std::bind(&PathPlannerNode::updateCurrPoseCallback, this, std::placeholders::_1));
    _arm_state_sub = this->create_subscription<std_msgs::msg::String>("arm_state", 1,
                            std::bind(&PathPlannerNode::updateStateCallback, this, std::placeholders::_1));
    _joint_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("arm_curr_angles", 1,
                            std::bind(&PathPlannerNode::joint_callback, this, std::placeholders::_1));


    _joint_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_target_joints", 1);
    _pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("arm_fk_pose", 1);

    moveit::core::RobotModelConstPtr robot_model_;
    robot_model_ = _move_group->getRobotModel();
    jmg = robot_model_->getJointModelGroup(_move_group->getName());
    robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state->setToDefaultValues();
}

void PathPlannerNode::receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const{
    // get trajectory plan based on target pos and publish joint targets 
    
    switch(_curr_state){
        case ArmState::IK: {
            // incremental update
            auto final_pose_target = std::make_shared<geometry_msgs::msg::Pose>();
            final_pose_target->orientation.x = _curr_pose->orientation.x + target_pose_msg->orientation.x;
            final_pose_target->orientation.y = _curr_pose->orientation.y + target_pose_msg->orientation.y;
            final_pose_target->orientation.z = _curr_pose->orientation.z + target_pose_msg->orientation.z;
            final_pose_target->orientation.w = _curr_pose->orientation.w + target_pose_msg->orientation.w;

            final_pose_target->position.x = _curr_pose->position.x + target_pose_msg->position.x;
            final_pose_target->position.y = _curr_pose->position.y + target_pose_msg->position.y;
            final_pose_target->position.z = _curr_pose->position.z + target_pose_msg->position.z;

            // check if still required, utils map_input_to_ik seems to handle it
            calculatePath(final_pose_target);
        }
        break;
        case ArmState::PATH_PLANNING: {
            calculatePath(target_pose_msg);
        }
        break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning not configured for state");
        break;
    }

    calculatePath(target_pose_msg);
}

void PathPlannerNode::joint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
 if (msg->data.size() != jmg->getVariableCount())
    {
        RCLCPP_ERROR(this->get_logger(),
          "Received %zu joint values but model expects %zu",
          msg->data.size(), jmg->getVariableCount());
        return;
    }

    // Convert float -> double
    std::vector<double> joint_positions(msg->data.begin(), msg->data.end());

    // Set joint positions
    robot_state->setJointGroupPositions(jmg, joint_positions);
    robot_state->update();

    // Compute FK
    Eigen::Isometry3d tf = robot_state->getGlobalLinkTransform("tool0");

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

    RCLCPP_INFO(this->get_logger(),
      "FK Published: [x=%.3f  y=%.3f  z=%.3f]",
      pose_msg.position.x,
      pose_msg.position.y,
      pose_msg.position.z);
}


void PathPlannerNode::calculatePath(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sub callback");

    // _move_group->setPoseTarget(*target_pose_msg);
    // auto const [success, plan] = [this]{
    //     moveit::planning_interface::MoveGroupInterface::Plan msg;
    //     auto const ok = static_cast<bool>(_move_group->plan(msg));
    //     return std::make_pair(ok, msg);
    // }();

    auto request = std::make_shared<arm_msgs::srv::PlanMotion::Request>();
    request->target_pose.pose = *target_pose_msg;
    request->target_pose.header.frame_id = "base_link";
    request->target_type = 1;
    request->planner_id = "RRTConnectkConfigDefault";

    RCLCPP_INFO(this->get_logger(),
            "pos = (%.3f, %.3f, %.3f)",
            request->target_pose.pose.position.x,
            request->target_pose.pose.position.y,
            request->target_pose.pose.position.z);

    using ServiceResponseFuture =
        rclcpp::Client<arm_msgs::srv::PlanMotion>::SharedFuture;

    auto callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service response: %s", response->message.c_str());

        if(response->success){
            publishPath(response->trajectory);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan published");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning failed");
        }
    };

    _client->async_send_request(request, callback);    
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
       	_joint_pose_pub->publish(msg);
    }
}