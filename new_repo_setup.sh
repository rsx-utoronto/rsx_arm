#!/bin/bash

# This script sets up the ROS 2 environment correctly for working on code.
# Because of environment issues with ROS 2, this script is necessary to ensure that colcon
# is setup up correctly to detect the packages in the virtual environment.

# It will additionally add an alias to the bashrc file to make it easier to make sure
# packages are built properly locally.

# It should not be run with ros, but with ./ instead. It assumes the ROS2 workspace is already created.

cd ../..
source /opt/ros/jazzy/setup.bash
sudo apt install -y python3-venv
python3 -m venv arm_env --system-site-packages --symlinks
source arm_env/bin/activate

cd src/rsx_arm

# This line reinstalls colcon-core and setuptools to ensure they use the correct Python environment.
pip install --force-reinstall colcon-core setuptools==$(pip list --no-index --format=json |  jq -r '.[] | select(.name=="setuptools").version')

# A helper command to enter the workspace
echo 'alias workon_arm="cd ~/arm_ros2_ws && source /opt/ros/jazzy/setup.bash && source arm_env/bin/activate && source install/setup.bash"' >> ~/.bashrc
workon_arm

# Another helper to build arm code
echo 'alias build_arm="workon_arm && colcon build"' >> ~/.bashrc
build_arm