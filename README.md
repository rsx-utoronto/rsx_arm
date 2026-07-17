# RSX's ARM Repository, now on ROS2!

Welcome to `rsx_arm`! This repository will host the ROS 2 (Humble) version of the RSX arm control software, migrated from the `arm/` folder in the old ROS 1 `rsx-rover` repo all the way back in the summer of '25. Proceed with caution. The following are computer setup instructions. For guidance on setting up and running the arm, see [arm_instructions.md](arm_instructions.md)

## Prerequisites

Make sure you have the following:

- **Ubuntu 22.04 LTS** 
- **ROS 2 Humble** installed per [these](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) instructions

## Workspace Setup
**Follow these instructions the first time you are cloning the repository.**

1. Create a new workspace
```bash
mkdir -p ~/arm_ros2_ws/src
cd ~/arm_ros2_ws
```
2. Clone this repository
```bash
cd src
git clone https://github.com/rsx-utoronto/rsx_arm.git
```

3. Run the `humble-setup.sh` script to get the environment set up!
```bash
cd rsx_arm
source humble-setup.sh
```
If the script prompts you with any confirmation (Y/n), type 'y' to let it proceed. The script also gives you access to the ```workon_arm``` and ```build_arm``` commands for a convenient way to, well, work on and build the arm. 😊 
**Note:** If you ever use `colcon build` to build packages, make sure you are in the workspace root (ie. `~/arm_ros2_ws`) first!

## Using IK

As of summer 2026, we utilize the trac_ik package to use IK with MoveIt. To get it set up:

1. Clone the repo below
```bash
cd ~/arm_ros2_ws/src
git clone https://github.com/rsx-utoronto/trac_ik_humble
```

2. Build and initialize with `rosdep`
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin
```

## Tracking Python Dependencies

The repo root contains a requirements.txt file which looks something like this: 

```
catkin_pkg
pyyaml
jinja2
typeguard
empy<4
...
```
Be sure to add any dependencies you use to this file as you code. 

Any time you use `git rebase` or `git pull` after major changes from other members, be sure to run:
```
pip install -r requirements.txt
```
This just helps us keep track of which packages we need and avoids any unexpected errors when running Python executables. 

Additionally, refrain from using ```pip freeze > requirements.txt``` to add new requirements. This seems to mistake our local ROS 2 packages as Python packages, leading to problems when someone else tries to install.

## Docker Workflow
The workflow is `env -> build -> up -> enter -> (inside container) cd src/rsx_arm -> deps -> setup_ws -> add_alias`

1. Setup the environment variables
```bash
# For Linux (Wayland)
source ./docker-scripts/env-linux.sh hyprland

# For Linux (X11)
source ./docker-scripts/env-linux.sh x11

# For WSL-G
source ./docker-scripts/env-wsl.sh
```

2. Build the image
```bash
# zsh + Neovim (default)
./docker-scripts/build.sh zsh nvim

# bash + VS Code (i.e., don’t install nvim)
./docker-scripts/build.sh bash vscode
```

3. Start the container.
```bash
# Linux + Hyprland (Wayland)
# Notes:
# Hyprland commonly wayland-1
# xhost +local: is used in wayland for using x11 when using Gazebo
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-1}"   
xhost +local:
./docker-scripts/up.sh hyprland

# Linux + X11
xhost +local:
./docker-scripts/up.sh x11

# macOS
./docker-scripts/up.sh mac

# Windows (PowerShell)
./docker-scripts/windows-up.ps1 -Profile windows -Shell zsh -Editor nvim

# Windows WSL (WSLG)
./docker-scripts/up.sh wsl
```

4. Enter the dev shell.
Choose service based on your platform.
- Linux: `rsxarm-x11`
- Linux (Wayland): `rsxarm-wayland`
- Mac: `rsxarm-mac`
- WSL: `rsxarm-wsl`
- Windows(PowerShell): `rsxarm-win`

```bash
# choose service if you used a profile name 
# (rsxarm, rsxarm-x11, rsxarm-wayland, rsxarm-mac, rsxarm-win, rsxarm-wsl)

./docker-scripts/enter.sh rsxarm
```

5. Inside the container: Install deps & build
```bash
./docker-scripts/deps.sh
./docker-scripts/setup_ws.sh
# run your nodes/launch files; ROS environment is sourced automatically by the entrypoint
```

6. Add helpful alias.
- `workon_arm`: `cd` into work dir, and source ROS. Note that this alias is redundant since sourcing is already done by Docker.
- `build_arm`: call `workon_arm`, then call `colcon build` and `colcon test`
```bash
./docker-scripts/add_alias.sh --both
```

If you want use `VSCode` (use `NeoVim` instead for better experience :) )
```bash
# For normal code
code .

# For X11
code-x11 .

# For Wayland
code-wayland .
```

### RealSense for WSL
On the Windows host, attach the camera to WSL with usbipd
```bash
usbipd wsl list
usbipd wsl attach --busid <BUSID> --distribution <YourWSLDistroName>
```

# Resources 
Feel free to add any resources here: 

1. Renard, E. (2024). ROS 2 from scratch : get started with ROS 2 and create robotics applications with Python and C++ (1st edition.). Packt Publishing Ltd. (Access through UofT library)
2. How to make a `requirements.txt`: https://www.geeksforgeeks.org/python/how-to-create-requirements-txt-file-in-python/
3. Python virtual environment stuff: https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb
