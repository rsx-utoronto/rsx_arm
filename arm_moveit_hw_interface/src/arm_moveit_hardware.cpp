#include "arm_moveit_hw_interface/arm_moveit_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_moveit_hardware {
    // implement each of the functions in hpp here later
}

// Make this class available as a plugin to the xacro file
PLUGINLIB_EXPORT_CLASS(
    arm_moveit_hardware::ArmSystemHardware,
    hardware_interface::SystemInterface
)