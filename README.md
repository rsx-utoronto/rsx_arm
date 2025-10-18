# RSX's ARM Repository, now on ROS2!

Welcome to `arm-ros2`! This repository will host the ROS 2 (Jazzy) version of the RSX arm control software, migrated from the `arm/` folder in the old ROS 1 `rsx-rover` repo. Proceed with caution.

## Prerequisites

Make sure you have the following:

- **Ubuntu 24.04 LTS** 
- **ROS 2 Jazzy** installed per [these](https://docs.ros.org/en/jazzy/Installation.html) instructions

Get the essential tools: 
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Initialize rosdep (once per machine):
```bash
sudo rosdep init
rosdep update
```

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

3. Run the new_repo_setup.sh script to get the environment set up!
```bash
cd rsx_arm
source new_repo_setup.sh
```
The final command above also gives you access to the ```workon_arm``` and ```build_arm``` commands for a convenient way to, well, work on and build the arm. 😊

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
And then, add any dependencies you discover as you port the code. Install them inside the venv with:

```
pip install -r requirements.txt
```

Each time you pull changes from the remote to your local machine, **MAKE SURE TO INSTALL THE REQUIREMENTS FILE!** It ensures you won't encounter a Python ```ImportError``` when you try and run a ROS 2 executable.

Additionally, although you may use ```pip freeze > requirements.txt``` to add new requirements, this seems to mistake our local ROS 2 packages as Python packages, leading to problems when someone else tries to install.

## Proposed Structure 

Below is a condensed directory tree of what the repo may look like:
```
rsx_arm/                   # repo root 
├── arm_msgs/              # ROS 2 CMake package with custom messages
│   ├── msg/
|   |   ├── ArmInputs.msg
│   ├── 
├── launch/
│   ├── arm.launch.py      # <- migrated launch file
├── controller/            # <- python ROS 2 package (as well as below)
├── gui/
├── simulation/
├── safety/
...
```
Important to note here is that each of the original folders were made into packages. Upon testing with ```colcon build``` we found that the directory structure shown in the old arm_ros2 repo would only allow files to be built if each folder contained an `__init__.py` file (or in other words, was a package).

## Proposed Migration Workflow

1. Analyze the Original `arm/` Folder and subfolders
- Identify ROS 1 Python nodes using `rospy`.
- Note topics, services, parameters, dependencies, and launch files.
- Document dependencies in `package.xml`. 

2. Set Up ROS 2 Packages
- Rename package (e.g., `controller`) and update metadata in package.xml and setup.py.
- Add necessary ROS 2 dependencies: `rclpy`, `std_msgs`, etc.

3. Migrate Nodes Incrementally
For each ROS 1 node:
- Create ROS 2 Python version using rclpy and ROS 2 APIs.
- Add entry point in the setup.py of the respective package:
```python
entry_points={
    'console_scripts': [
        'arm_controller = arm_ros2.arm_controller:main',
    ],
},

```
- Build, source, and test with:
```bash
build_arm
ros2 run <package_name> <executable_name>
```

4. Migrate Launch Files
Convert .launch to .launch.py, using ROS 2 conventions:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_ros2', executable='mynode', name='mynode'),
    ])
```

5. Add Custom Messages/Services/Actions
If migrating custom `msg/`, `srv/`, or `action/` files:
- Place definitions in the respective ROS 2 directories.
- Update `package.xml` and `CMakeLists.txt`.
- Rebuild and import accordingly in nodes.

6. **If moving files you have already worked on in arm_ros2 to rsx_arm**
If you are placing a file into one of the python packages:
- Go into that package's folder (ie. `controller`)
- Switch into the nested directory with that same name (ie. `controller/controller`)
- Add the file there (in the folder with `__init__.py`)
- Modify the `package.xml` AND `requirements.txt` files to account for new dependencies
- Add to the console_scripts list in the setup.py file to let ROS 2 detect your executable using the following convention:
```python3
'executable_name = package_name.node_name:function',
```

## Team Workflow
- Work only in this repository (`rsx_arm`); keep `rsx-rover` as a reference.
- Use feature branches for each node migration.
- Submit PRs and conduct peer reviews before merging.
- After migrating each node:
    - Reinstall requirements and build (in the repo root):
      ```bash
      pip install -r requirements.txt
      build_arm
      ```
    - Source:
      ```bash
      workon_arm
      ```
    - Run & test:
      ```bash
      ros2 run arm_ros2 <node>
      ros2 topic echo /<topic>
      ```
- Track progress with an issue board (e.g., "Migrate node X", "Convert launch file Y").

## Docker Workflow
1. Build the image
```bash
# zsh + Neovim (default)
./docker-scripts/build.sh zsh nvim

# bash + VS Code (i.e., don’t install nvim)
./docker-scripts/build.sh bash vscode
```

2. Start the container.
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

3. Enter the dev shell.
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

4. Inside the container: Install deps & build
```bash
./docker-scripts/deps.sh
./docker-scripts/setup_ws.sh
# run your nodes/launch files; ROS environment is sourced automatically by the entrypoint
```

5. Add helpful alias.
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

# Collaboration Code of Conduct:

  1. Engineering Specification and Background Research (FOCs, Research into task)
  2. Candidate Designs
      - Don’t need a final design, block diagrams with explanation are fine
  3. Final Design
      - Can be one of the previous two designs or something all new based on learned
      - Can be a block diagram with research and explanation

# Resources 
Feel free to add any resources here: 

1. Renard, E. (2024). ROS 2 from scratch : get started with ROS 2 and create robotics applications with Python and C++ (1st edition.). Packt Publishing Ltd. (Access through UofT library)
2. How to make a `requirements.txt`: https://www.geeksforgeeks.org/python/how-to-create-requirements-txt-file-in-python/
3. Python virtual environment stuff: https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb
