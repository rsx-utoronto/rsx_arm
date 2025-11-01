#ifndef ARM_MOVEIT_HARDWARE
#define ARM_MOVEIT_HARDWARE

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_moveit_hardware {
    class ArmSystemHardware : public hardware_interface::Systeminterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware); // Defines ROS2 types commonly used (ie. SharedPtr) but for this class

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
    };
} // arm_moveit_hardware namespace

#endif // ARM_MOVEIT_HARDWARE