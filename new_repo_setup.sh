#!/bin/bash

# This script sets up the ROS 2 environment correctly for working on code.
# Because of environment issues with ROS 2, this script is necessary to ensure that colcon
# is setup up correctly to detect the packages in the virtual environment.

# It will additionally add an alias to the bashrc file to make it easier to make sure
# packages are built properly locally.

# It should not be run with ros, but with source instead. It assumes the ROS2 workspace is already created.

cd ../.. && \
source /opt/ros/jazzy/setup.bash && \
echo "Setup requires sudo privileges for installing python3-venv." && \
sudo apt install -y python3-venv && \
python3 -m venv arm_env --system-site-packages --symlinks && \
source arm_env/bin/activate && \

# This line reinstalls colcon-core and setuptools to ensure they use the correct Python environment.
pip install --force-reinstall colcon-core setuptools==$(pip list --no-index --format=json |  jq -r '.[] | select(.name=="setuptools").version') && \

# Build the workspace for the first time
colcon build && \

# A helper command to enter the workspace
alias workon_arm="cd $(pwd) && source /opt/ros/jazzy/setup.bash && source arm_env/bin/activate && source install/setup.bash" && \
echo 'alias workon_arm="cd $(pwd) && source /opt/ros/jazzy/setup.bash && source arm_env/bin/activate && source install/setup.bash"' >> ~/.bashrc && \
workon_arm && \

# Another helper to build arm code
alias build_arm="workon_arm && colcon build" && \
echo 'alias build_arm="workon_arm && colcon build"' >> ~/.bashrc && \
build_arm && \

# Make sure colcon doesn't try to build the virtual environment
touch arm_env/COLCON_IGNORE && \

# Go back to the source directory and install pip requirements
cd src/rsx_arm && \
pip install -r requirements.txt && \

# A final message to indicate completion
echo "" && \
echo "Setup complete. In the future, use 'workon_arm' to enter the workspace and 'build_arm' to build the packages." && \
echo "Use the \"deactivate\" command to exit the virtual environment when done."