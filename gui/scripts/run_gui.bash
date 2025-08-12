#!/bin/bash

# To give the GUI access to the source directory for icon and .yaml from anywhere
export GUI_DIR="$HOME/arm_ros2_ws/src/rsx_arm/gui/gui/"
echo $gui_dir

# To actually run stuff
ros2 run gui arm_gui &
ros2 launch arm_launch arm_rviz_launch.py ik_on:=true