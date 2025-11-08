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
  
    _joint_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_target_joints", 1);

    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_service_client");
    _client = this->create_client<arm_msgs::srv::PlanMotion>("/planner_server/plan_motion");


}

void PathPlannerNode::receiveTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr target_pose_msg) const{
    // get trajectory plan based on target pos and publish joint targets 
    // _curr_state =ArmState::PATH_PLANNING;   
    // switch(_curr_state){
    //     case ArmState::IK: {
    //         // incremental update
    //         auto final_pose_target = std::make_shared<geometry_msgs::msg::Pose>();
    //         final_pose_target->orientation.x = _curr_pose->orientation.x + target_pose_msg->orientation.x;
    //         final_pose_target->orientation.y = _curr_pose->orientation.y + target_pose_msg->orientation.y;
    //         final_pose_target->orientation.z = _curr_pose->orientation.z + target_pose_msg->orientation.z;
    //         final_pose_target->orientation.w = _curr_pose->orientation.w + target_pose_msg->orientation.w;

    //         final_pose_target->position.x = _curr_pose->position.x + target_pose_msg->position.x;
    //         final_pose_target->position.y = _curr_pose->position.y + target_pose_msg->position.y;
    //         final_pose_target->position.z = _curr_pose->position.z + target_pose_msg->position.z;

    //         calculatePath(final_pose_target);
    //     }
    //     break;
    //     case ArmState::PATH_PLANNING: {
    //         calculatePath(target_pose_msg);
    //     }
    //     break;
    // }
    calculatePath(target_pose_msg);

    
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
    request->target_pose.pose.orientation.x  = 0.4;

    while (!_client->wait_for_service(5s)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "can't connect");

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //   return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

    auto result = _client->async_send_request(request);
    auto trajectory = result.get()->trajectory;
    std::cout << result.get()->message << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), result.get()->message.c_str());
    publishPath(trajectory);
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