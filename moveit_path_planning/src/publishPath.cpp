#include <memory>
#include <chrono>
#include "../include/PathPlannerNode.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto const node = std::make_shared<rclcpp::Node>(
		"path_planning",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

	// ROS logger
	auto logger = rclcpp::get_logger("path_planning");

	// constant or input?
	const std::string PLANNING_GROUP = "rover_arm"; 
	const std::string target_pose_topic = "arm_target_pose";
	const std::string target_joints_topic = "arm_target_joints";

	using moveit::planning_interface::MoveGroupInterface;
	auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);


	auto planner_node = std::make_shared<PathPlannerNode>(&move_group_interface, target_pose_topic, target_joints_topic);
	rclcpp::spin(planner_node);
	rclcpp::shutdown();
	return 0;
}