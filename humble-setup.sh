#!/bin/bash

# This script sets up the ROS 2 environment correctly for working on code.
# Because of environment issues with ROS 2, this script is necessary to ensure that colcon
# is setup up correctly to detect the packages in the virtual environment.

# It will additionally add an alias to the bashrc file to make it easier to make sure
# packages are built properly locally.

# It should not be run with ros, but with source instead. It assumes the ROS2 workspace is already created.

# Get workspace root
cd ../.. 
workspace_dir=$(pwd)

env_name=arm_env

source /opt/ros/humble/setup.bash 
echo -e "\e[32mSetup requires sudo privileges for installing required packages.\e[0m" 
sudo snap install jq 
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo apt install python3-evdev python3-pip
sudo apt install python3.10-venv  
python3.10 -m venv $env_name --system-site-packages --symlinks 
source $workspace_dir/$env_name/bin/activate 

# Make sure colcon doesn't try to build the virtual environment
touch $workspace_dir/$env_name/COLCON_IGNORE

# Initialize + update using rosdep
sudo rosdep init
rosdep update

# Go back to the source directory and install pip requirements 
cd $workspace_dir/src/rsx_arm 
pip install -r requirements.txt 

# This line reinstalls colcon-core and setuptools to ensure they use the correct Python environment.
pip install --force-reinstall colcon-core setuptools==$(pip list --no-index --format=json |  jq -r '.[] | select(.name=="setuptools").version') 
# Colcon installs empy 4.2 by default, which is incompatible with Humble, so we uninstall it, use 3.3.4 instead
pip uninstall empy==4.2  

# Go back down to workspace root
cd $workspace_dir

# Build the workspace for the first time, but build arm_msgs first as the rest of packages depend on it!
colcon build --packages-select arm_msgs

# A helper command to enter the workspace
echo -e "\nalias workon_arm='cd $workspace_dir && source /opt/ros/humble/setup.bash && source $env_name/bin/activate && source install/setup.bash'" >> ~/.bashrc 

# Another helper to build arm code
echo -e "\nalias build_arm='workon_arm && colcon build --symlink-install && colcon test --ctest-args tests --packages-skip arm_msgs && colcon test-result --all --verbose'" >> ~/.bashrc 
echo -e "\nalias format_code='autopep8 --in-place --recursive src/rsx_arm'" >> ~/.bashrc 
# Build the arm (this also runs workon_arm for us)
source ~/.bashrc && build_arm  

# A final message to indicate completion
echo "" 
echo -e "\e[32mSetup complete. In the future, use 'workon_arm' to enter the workspace and 'build_arm' to build the packages.\e[0m" 
echo -e "\e[32mUse the \"deactivate\" command to exit the virtual environment when done.\e[0m" 

cd $workspace_dir/src/rsx_arm
