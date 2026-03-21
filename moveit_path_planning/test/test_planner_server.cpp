#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "arm_msgs/srv/plan_motion.hpp"

// Test fixture for planner server tests
class PlannerServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_planner_server");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
};

// Basic initialization test
TEST_F(PlannerServerTest, NodeInitialization)
{
  ASSERT_NE(node_, nullptr);
  EXPECT_EQ(node_->get_name(), std::string("test_planner_server"));
}

// Test service request message construction
TEST_F(PlannerServerTest, ServiceRequestConstruction)
{
  auto request = std::make_shared<arm_msgs::srv::PlanMotion::Request>();
  
  // Test joint target
  request->target_type = arm_msgs::srv::PlanMotion::Request::TARGET_TYPE_JOINT;
  request->joint_names = {"joint_1", "joint_2", "joint_3"};
  request->joint_values = {0.0, 0.5, -0.5};
  request->planner_id = "RRTConnectkConfigDefault";
  request->planning_time = 5.0;
  request->num_planning_attempts = 1;
  request->velocity_scaling_factor = 0.1;
  request->acceleration_scaling_factor = 0.1;
  
  EXPECT_EQ(request->target_type, 0);
  EXPECT_EQ(request->joint_names.size(), 3);
  EXPECT_EQ(request->joint_values.size(), 3);
  EXPECT_EQ(request->planner_id, "RRTConnectkConfigDefault");
}

// Test deterministic mode parameters
TEST_F(PlannerServerTest, DeterministicMode)
{
  auto request = std::make_shared<arm_msgs::srv::PlanMotion::Request>();
  
  request->deterministic = true;
  request->random_seed = 12345;
  
  EXPECT_TRUE(request->deterministic);
  EXPECT_EQ(request->random_seed, 12345u);
}

// Test cartesian waypoint request construction
TEST_F(PlannerServerTest, CartesianWaypointRequest)
{
  auto request = std::make_shared<arm_msgs::srv::PlanMotion::Request>();
  
  request->target_type = arm_msgs::srv::PlanMotion::Request::TARGET_TYPE_CARTESIAN;
  
  geometry_msgs::msg::PoseStamped waypoint1;
  waypoint1.header.frame_id = "base_link";
  waypoint1.pose.position.x = 0.3;
  waypoint1.pose.position.y = 0.0;
  waypoint1.pose.position.z = 0.5;
  waypoint1.pose.orientation.w = 1.0;
  
  geometry_msgs::msg::PoseStamped waypoint2;
  waypoint2.header.frame_id = "base_link";
  waypoint2.pose.position.x = 0.4;
  waypoint2.pose.position.y = 0.1;
  waypoint2.pose.position.z = 0.6;
  waypoint2.pose.orientation.w = 1.0;
  
  request->waypoints.push_back(waypoint1);
  request->waypoints.push_back(waypoint2);
  request->cartesian_max_step = 0.01;
  request->cartesian_jump_threshold = 0.0;
  
  EXPECT_EQ(request->waypoints.size(), 2);
  EXPECT_DOUBLE_EQ(request->cartesian_max_step, 0.01);
}

// Test response validation
TEST_F(PlannerServerTest, ResponseValidation)
{
  auto response = std::make_shared<arm_msgs::srv::PlanMotion::Response>();
  
  response->success = true;
  response->error_code = 1;  // SUCCESS
  response->message = "Planning succeeded";
  response->planning_time = 1.234;
  response->waypoints_count = 50;
  response->planner_used = "RRTConnectkConfigDefault";
  
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->error_code, 1);
  EXPECT_GT(response->planning_time, 0.0);
  EXPECT_GT(response->waypoints_count, 0u);
  EXPECT_FALSE(response->planner_used.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
