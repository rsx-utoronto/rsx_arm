#include <memory>
#include<chrono>
#include "rclcpp/rclcpp.hpp"
#include <moveit_path_planning/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include "arm_msgs/srv/plan_motion.hpp"
#include "arm_msgs/action/plan_and_execute.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#define NUM_JOINTS 6

class CommandNOde : public rclcpp::Node
{
  private:
  //