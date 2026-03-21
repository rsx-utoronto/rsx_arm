#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "moveit_path_planning/planner_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<moveit_path_planning::PlannerServer>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
