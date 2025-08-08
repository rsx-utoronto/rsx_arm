#!/bin/bash

# This script sets up the ROS 2 environment correctly for working on code.
# Because of environment issues with ROS 2, this script is necessary to ensure that colcon
# is setup up correctly to detect the packages in the virtual environment.

# It will additionally add an alias to the bashrc file to make it easier to make sure
# packages are built properly locally.

# It should not be run with ros, but with source instead. It assumes the ROS2 workspace is already created.

cd ../.. && \
source /opt/ros/humble/setup.bash && \
echo "Setup requires sudo privileges for installing python3-venv." && \
# Ensure Python 3.12 is installed and set up the virtual environment, Ubuntu 22.04 does not carry 3.12 by default
sudo add-apt-repository ppa:deadsnakes/ppa -y && sudo apt update && sudo apt upgrade -y &&  sudo apt install -y python3.12 python3.12-venv && \
sudo snap install jq && \
sudo apt install python3-evdev && \
python3.12 -m venv arm_env --system-site-packages --symlinks && \
source ~/arm_ros2_ws/arm_env/bin/activate && \

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
colcon build && \

workspace_dir=$(pwd) && \

# A helper command to enter the workspace
echo -e "\nalias workon_arm='cd $workspace_dir && source /opt/ros/humble/setup.bash && source arm_env/bin/activate && source install/setup.bash'" >> ~/.bashrc && \

# Another helper to build arm code
echo -e "\nalias build_arm='workon_arm && colcon build'" >> ~/.bashrc && \

# Run the two helper commands
source ~/.bashrc && workon_arm && build_arm && \

# Make sure colcon doesn't try to build the virtual environment
touch arm_env/COLCON_IGNORE && \

# A final message to indicate completion
echo "" && \
echo "Setup complete. In the future, use 'workon_arm' to enter the workspace and 'build_arm' to build the packages." && \
echo -e "Use the \"deactivate\" command to exit the virtual environment when done." && \

cd src/rsx_arm
