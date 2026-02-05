#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class IkToRvizJointState : public rclcpp::Node
{
public:
  static constexpr size_t NUM_JOINTS = 6;

  IkToRvizJointState()
  : Node("ik_to_rviz_joint_state")
  {
    // Publisher to RViz joint_states topic
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscriber to IK target joints topic
    ik_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "safe_arm_target_joints", 10,
      std::bind(&IkToRvizJointState::to_rviz_cb, this, std::placeholders::_1));

    // Subscriber to path planning target joints topic
    path_planning_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "arm_path_joints", 10,
      std::bind(&IkToRvizJointState::to_rviz_cb, this, std::placeholders::_1));

    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  }

private:
  void to_rviz_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < NUM_JOINTS) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joints array has %zu values; expected %zu",
        msg->data.size(), NUM_JOINTS);
      return;
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();     // RViz-friendly timestamp
    js.name = joint_names_;
    js.position.resize(NUM_JOINTS);

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      js.position[i] = static_cast<double>(msg->data[i])*3.14/180.0;  // Convert degrees to radians
    }

    joint_pub_->publish(js);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ik_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_planning_sub_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkToRvizJointState>());
  rclcpp::shutdown();
  return 0;
}
