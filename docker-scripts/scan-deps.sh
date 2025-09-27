#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-.}"

echo "=== ROS msgs used (implies package deps) ==="
rg -n --no-messages -g '!build/**' -e '([a-zA-Z_]+)_msgs/msg/' "$ROOT" | sed -E 's#.*/([A-Za-z0-9_]+)_msgs/msg/.*#\1_msgs#' | sort -u

echo; echo "=== C++ includes that imply system libs ==="
rg -n --no-messages -g '!build/**' -e '#include <opencv2/' -e '#include <pcl/' -e '#include <Eigen/' -e '#include <yaml-cpp/' "$ROOT" || true

echo; echo "=== CMake find_package / ament_target_dependencies ==="
rg -n --no-messages -g '!build/**' -e 'find_package\(' -e 'ament_target_dependencies\(' "$ROOT" || true

echo; echo "=== Python imports of interest (map to apt/pip) ==="
rg -n --no-messages -g '!build/**' -e '^\s*(from|import)\s+(cv2|cv_bridge|PIL|numpy|scipy|skimage|matplotlib|serial|yaml|rclpy)\b' "$ROOT" || true

echo; echo "=== launch files referencing packages ==="
rg -n --no-messages -g '!build/**' -e 'pkg=|package=' -e 'launch_ros.actions.Node' "$ROOT" || true

cat <<'MAP'

--- Suggested mapping (put these in package.xml; rosdep handles apt names) ---
cv_bridge           -> <depend>cv_bridge</depend>            # becomes ros-humble-cv-bridge
cv2 (Python)        -> <exec_depend>python3-opencv</exec_depend>
OpenCV C++ includes -> <build_depend>libopencv-dev</build_depend> <exec_depend>libopencv-dev</exec_depend>
Pillow (PIL)        -> <exec_depend>python3-pil</exec_depend>     # OR pip 'Pillow' (prefer apt with ROS)
numpy               -> <exec_depend>python3-numpy</exec_depend>
pyserial            -> <exec_depend>python3-serial</exec_depend>
yaml (PyYAML)       -> <exec_depend>python3-yaml</exec_depend>
matplotlib          -> <exec_depend>python3-matplotlib</exec_depend>
scipy               -> <exec_depend>python3-scipy</exec_depend>
pcl headers         -> <build_depend>libpcl-dev</build_depend>
Eigen headers       -> <build_depend>libeigen3-dev</build_depend>
yaml-cpp headers    -> <build_depend>libyaml-cpp-dev</build_depend>
tf2 / tf2_ros       -> <depend>tf2_ros</depend> (+ tf2_geometry_msgs if used)
image_transport     -> <depend>image_transport</depend>
sensor_msgs/...     -> <depend>sensor_msgs</depend>
geometry_msgs/...   -> <depend>geometry_msgs</depend>
nav_msgs/...        -> <depend>nav_msgs</depend>
MAP
