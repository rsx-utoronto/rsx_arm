#!/bin/bash

# This script sets up the ROS 2 environment correctly for working on code.
# Because of environment issues with ROS 2, this script is necessary to ensure that colcon
# is setup up correctly to detect the packages in the virtual environment.

# It will additionally add an alias to the bashrc file to make it easier to make sure
# packages are built properly locally.

# It should not be run with ros, but with source instead. It assumes the ROS2 workspace is already created.

cd ~rover_ws && \
source /opt/ros/humble/setup.bash && \
# Go back to the source directory and install pip requirements 
cd src/rsx_arm && \
pip install -r requirements.txt && \
# This line reinstalls colcon-core and setuptools to ensure they use the correct Python environment.
pip install --force-reinstall colcon-core setuptools==$(pip list --no-index --format=json |  jq -r '.[] | select(.name=="setuptools").version') && \
# Colcon installs empy 4.2 by default, which is incompatible with Humble, so we uninstall it, use 3.3.4 instead
pip uninstall empy==4.2 && \ 

# Go back down to workspace root
cd ../.. && \

# Build the workspace for the first time
colcon build --packages-select arm_controller arm_launch arm_moveit_config arm_msgs arm_utilities auto_keyboard gui moveit_path_planning&& \

# Another helper to build arm code
echo -e "\nalias build_arm='workon_arm && colcon build --packages-select arm_controller arm_launch arm_moveit_config arm_utilities auto_keyboard gui moveit_path_planning --packages-skip arm_msgs" >> ~/.bashrc && \

# Run the two helper commands
source ~/.bashrc && build_arm && \
