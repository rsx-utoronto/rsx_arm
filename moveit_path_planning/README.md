# MoveIt Path Planning Module

A complete ROS 2 path planning module integrated with MoveIt 2 for the rsx_arm project. Provides collision-free trajectory planning with support for joint-space targets, Cartesian poses, and waypoint paths.

## Features

- **Multiple Planning Modes**
  - Joint-space targets: Plan to specific joint configurations
  - Pose targets: Plan to Cartesian poses with IK solving
  - Cartesian paths: Plan through waypoints with collision checking

- **Flexible Planning Configuration**
  - Multiple OMPL planners (RRTConnect, RRT*, PRM*, etc.)
  - Configurable velocity and acceleration scaling
  - Path constraints and goal tolerances
  - Deterministic planning mode (reproducible results)

- **Time Parameterization**
  - Iterative Parabolic Time Parameterization (IPTP) - default
  - Time-Optimal Trajectory Generation (TOTG) - optional

- **Execution Support**
  - Optional trajectory execution via FollowJointTrajectory action
  - Real-time feedback during planning and execution
  - Trajectory validation before execution

- **ROS 2 Interfaces**
  - Service: `PlanMotion.srv` - fast planning without execution
  - Action: `PlanAndExecute.action` - planning with optional execution

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Planner Server                       │
│  ┌──────────────────────────────────────────────────┐  │
│  │         MoveGroupInterface                       │  │
│  │  ┌────────────┐  ┌────────────┐  ┌───────────┐ │  │
│  │  │ Joint Plan │  │ Pose Plan  │  │ Cartesian │ │  │
│  │  └────────────┘  └────────────┘  └───────────┘ │  │
│  └──────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────┐  │
│  │       PlanningSceneMonitor                       │  │
│  │  (collision detection, obstacle tracking)        │  │
│  └──────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────┐  │
│  │    Time Parameterization (IPTP/TOTG)            │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│            FollowJointTrajectory Controller             │
│                  (execution bridge)                      │
└─────────────────────────────────────────────────────────┘
```

## Installation & Build

### Prerequisites

- ROS 2 Humble or Jazzy
- MoveIt 2
- arm_moveit_config package
- arm_msgs package

### Build Instructions

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # or jazzy

# Navigate to workspace
cd ~/arm_ros2_ws

# Build packages
colcon build --packages-select arm_msgs moveit_path_planning

# Source workspace
source install/setup.bash
```

## Usage

### Launch Planner Server

Start the planner server node with MoveIt configuration:

```bash
ros2 launch moveit_path_planning planner_server.launch.py
```

### Launch RViz Demo

Start complete stack with RViz visualization:

```bash
ros2 launch moveit_path_planning rviz_planning_demo.launch.py
```

This launches:
- move_group (MoveIt planning pipeline)
- planner_server (planning service/action)
- RViz with MoveIt plugin
- Static TF publishers

### Using the Service Interface

#### Joint Target Planning

```bash
ros2 service call /planner_server/plan_motion arm_msgs/srv/PlanMotion "{
  target_type: 0,
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  joint_values: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
  planner_id: 'RRTConnectkConfigDefault',
  planning_time: 5.0,
  velocity_scaling_factor: 0.1
}"
```

#### Pose Target Planning

```bash
ros2 service call /planner_server/plan_motion arm_msgs/srv/PlanMotion "{
  target_type: 1,
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.3, y: 0.2, z: 0.5},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  planner_id: 'RRTConnectkConfigDefault',
  planning_time: 5.0
}"
```

### Python Client Examples

Simple service call:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_msgs.srv import PlanMotion

rclpy.init()
node = Node('simple_planner_client')
client = node.create_client(PlanMotion, '/planner_server/plan_motion')
client.wait_for_service()

request = PlanMotion.Request()
request.target_type = PlanMotion.Request.TARGET_TYPE_JOINT
request.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
request.joint_values = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
request.planning_time = 5.0

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

response = future.result()
if response.success:
    print(f"Planning succeeded! Waypoints: {response.waypoints_count}")
else:
    print(f"Planning failed: {response.message}")

node.destroy_node()
rclpy.shutdown()
```

Full examples with multiple modes:

```bash
# Run service example (joint, pose, Cartesian, deterministic)
python3 src/moveit_path_planning/scripts/example_planner_client.py

# Run action example (planning with optional execution)
python3 src/moveit_path_planning/scripts/example_action_client.py
```

## API Reference

### Service: PlanMotion

**Request Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `target_type` | uint8 | 0=JOINT, 1=POSE, 2=CARTESIAN |
| `joint_names` | string[] | Joint names (for JOINT mode) |
| `joint_values` | float64[] | Joint values (for JOINT mode) |
| `target_pose` | PoseStamped | Target pose (for POSE mode) |
| `waypoints` | PoseStamped[] | Waypoints (for CARTESIAN mode) |
| `cartesian_max_step` | float64 | Max step size (default: 0.01 m) |
| `cartesian_jump_threshold` | float64 | Jump threshold (default: 0.0) |
| `frame_id` | string | Reference frame |
| `path_constraints` | Constraints | MoveIt path constraints |
| `planner_id` | string | OMPL planner (default: RRTConnect) |
| `planning_time` | float64 | Timeout in seconds (default: 5.0) |
| `num_planning_attempts` | uint32 | Attempts (default: 1) |
| `velocity_scaling_factor` | float64 | 0.0-1.0 (default: 0.1) |
| `acceleration_scaling_factor` | float64 | 0.0-1.0 (default: 0.1) |
| `deterministic` | bool | Use seeded RNG (default: false) |
| `random_seed` | uint64 | Seed for deterministic mode |

**Response Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Planning succeeded |
| `error_code` | int32 | MoveIt error code |
| `message` | string | Status message |
| `trajectory` | RobotTrajectory | Planned trajectory |
| `planning_time` | float64 | Actual planning time |
| `waypoints_count` | uint32 | Number of trajectory points |
| `planner_used` | string | Planner that was used |

### Action: PlanAndExecute

**Goal:** Same as PlanMotion request + `execute: bool`

**Result:** Same as PlanMotion response + execution status

**Feedback:**

| Field | Type | Description |
|-------|------|-------------|
| `stage` | string | "planning", "validating", "executing" |
| `percent_complete` | float64 | Progress 0.0-100.0 |
| `status_message` | string | Current status |

## Parameters

Configure via `config/planner_server.yaml` or command line:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `planning_group` | string | "rover_arm" | MoveIt planning group |
| `base_frame` | string | "base_link" | Base reference frame |
| `ee_frame` | string | "end_effector_link" | End-effector frame |
| `default_planner_id` | string | "RRTConnectkConfigDefault" | Default OMPL planner |
| `default_planning_time` | double | 5.0 | Default timeout (seconds) |
| `default_num_attempts` | int | 1 | Default planning attempts |
| `default_velocity_scaling` | double | 0.1 | Default velocity scaling |
| `default_acceleration_scaling` | double | 0.1 | Default acceleration scaling |
| `default_cartesian_step` | double | 0.01 | Cartesian path step size (m) |
| `default_cartesian_jump_threshold` | double | 0.0 | Jump threshold |
| `use_time_optimal_parameterization` | bool | false | Use TOTG vs IPTP |

Override parameters:

```bash
ros2 launch moveit_path_planning planner_server.launch.py \
  default_planner_id:=RRTstarkConfigDefault \
  default_planning_time:=10.0
```

## Available OMPL Planners

Configured in `arm_moveit_config/config/ompl_planning.yaml`:

- **RRTConnectkConfigDefault** (default) - Fast, bidirectional
- **RRTstarkConfigDefault** - Optimal, asymptotically optimal
- **PRMstarkConfigDefault** - Roadmap-based, optimal
- **RRTkConfigDefault** - Simple RRT
- **PRMkConfigDefault** - Probabilistic roadmap
- And many more...

## Testing

### Run Unit Tests

```bash
colcon test --packages-select moveit_path_planning
colcon test-result --verbose
```

### Manual Testing

1. Start planner server:
   ```bash
   ros2 launch moveit_path_planning planner_server.launch.py
   ```

2. In another terminal, run example client:
   ```bash
   python3 src/moveit_path_planning/scripts/example_planner_client.py
   ```

3. Verify trajectory visualization in RViz (if running demo launch)

## Deterministic Planning

For reproducible results (e.g., testing, benchmarking):

```python
request.deterministic = True
request.random_seed = 12345  # Use same seed for identical results
```

This seeds the OMPL RNG, ensuring the same planning problem produces the same trajectory.

## Troubleshooting

### Service/Action Not Available

**Issue:** `ros2 service list` doesn't show `/planner_server/plan_motion`

**Solution:**
- Ensure planner_server is running: `ros2 node list`
- Check for errors in planner_server logs
- Verify arm_msgs package is built and sourced

### Planning Always Fails

**Issue:** All planning requests return `success: false`

**Solution:**
- Check that robot_description is loaded: `ros2 param get /planner_server robot_description`
- Verify planning group exists: `ros2 param get /planner_server planning_group`
- Ensure target is reachable and collision-free
- Increase `planning_time` or `num_planning_attempts`

### Execution Fails

**Issue:** Planning succeeds but execution fails

**Solution:**
- Verify controller is running: `ros2 control list_controllers`
- Check action server: `ros2 action list | grep follow_joint_trajectory`
- Ensure robot is in safe starting state
- Check for collision warnings in logs

### Build Errors

**Issue:** `arm_msgs/srv/plan_motion.hpp: No such file`

**Solution:**
```bash
# Build arm_msgs first
colcon build --packages-select arm_msgs
source install/setup.bash
# Then build moveit_path_planning
colcon build --packages-select moveit_path_planning
```

## Integration with Existing Systems

This module is designed to integrate seamlessly with:

- **arm_moveit_config**: Uses URDF, SRDF, kinematics, and OMPL configs
- **controller**: Can execute via FollowJointTrajectory
- **visualization**: Trajectories visible in RViz MoveIt plugin
- **safety**: Planning respects collision geometry
- **simulation**: Works in Gazebo with `use_sim_time:=true`

## Performance Notes

- **Joint planning**: ~0.1-2.0 seconds (depends on planner, complexity)
- **Pose planning**: ~0.5-5.0 seconds (includes IK solving)
- **Cartesian paths**: ~0.1-1.0 seconds (depends on waypoint count)
- **Deterministic mode**: Same planning time, reproducible results

Tips for faster planning:
- Use RRTConnect for speed (default)
- Reduce `planning_time` for acceptable solutions
- Increase `velocity_scaling_factor` if safe
- Use joint targets when possible (faster than pose targets)

## License

See the LICENSE file in the repository root.

## Changelog

### Version 1.0.0 (2025-11-07)

- Initial release
- Service interface (PlanMotion)
- Action interface (PlanAndExecute)
- Support for joint, pose, and Cartesian planning
- Deterministic planning mode
- Time parameterization (IPTP/TOTG)
- Trajectory validation
- Optional execution support
- Launch files and RViz demo
- Unit tests and examples
- Comprehensive documentation
