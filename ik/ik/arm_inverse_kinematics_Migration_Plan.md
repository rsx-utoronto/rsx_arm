# Migration Plan: `arm_inverse_kinematics.py` (ROS 1 to ROS 2)

This document outlines the code parts from the original ROS 1 file `arm_inverse_kinematics.py` that have already been migrated into the new ROS 2 version, as well as those that remain to be implemented. It also explains how to best structure and include the missing functionalities.

---

## ✅ Already Migrated to ROS 2

### 1. ROS 2 Node Initialization

- **ROS 1**:
  ```python
  rospy.init_node("arm_ik")
  ```
- **ROS 2**:
  ```python
  rclpy.init()
  node = InverseKinematicsNode()
  rclpy.spin(node)
  ```

### 2. Publishers

- **Topics**:

  - `arm_goal_pos`
  - `arm_end_effector_pos`

- **Migrated Code**:

  ```python
  self.armAnglesPub = self.create_publisher(Float32MultiArray, "arm_goal_pos", 10)
  self.armPosPub = self.create_publisher(Float32MultiArray, "arm_end_effector_pos", 10)
  ```

### 3. Subscribers

- **Topics**:

  - `arm_curr_pos`
  - `arm_state`
  - `arm_inputs`
  - `ik_targets`

- **Migrated Code**:

  ```python
  self.create_subscription(Float32MultiArray, "arm_curr_pos", self.updateLiveArmAngles, 10)
  self.create_subscription(String, "arm_state", self.onStateUpdate, 10)
  self.create_subscription(ArmInputs, "arm_inputs", self.onControllerUpdate, 10)
  self.create_subscription(Float32MultiArray, "ik_targets", self.onIKTargetUpdate, 10)
  ```

### 4. TF Broadcasting

- ROS 1 used `tf_conversions` and `tf2_ros.TransformBroadcaster()`.
- ROS 2 uses:
  ```python
  from tf2_ros import TransformBroadcaster
  self.br = TransformBroadcaster(self)
  ```

### 5. Node Loop

- Replaced ROS 1 spin loop with ROS 2 timer:
  ```python
  self.create_timer(1.0 / self.rate_hz, self.run_main_loop)
  ```

### 6. Internal State Tracking

- Variables like `self.curArmAngles`, `self.angleCorrections`, and `self.scriptMode` structure have been preserved.

---

## ⛔ Not Yet Migrated (To Do)

### 1. **Service Definitions**

- ROS 1 Services:

  - `update_arm_corrections` → `Corrections`
  - `move_arm_to` → `GoToArmPos`
  - `save_arm_pos_as` → `SaveArmPos`

- **What To Do:**

  - Convert to ROS 2 format:
    ```python
    self.create_service(Corrections, 'update_arm_corrections', self.updateAngleCorrections)
    ```
  - Ensure `.srv` files are compiled in ROS 2 using `rosidl_interface_packages`

### 2. **ScriptMode Classes**

- Classes like `ForwardKin`, `DefaultIK`, `GlobalIK`, etc., are currently missing.

- **What To Do:**

  - Move each `ScriptMode` class into its own module, e.g.:
    - `scripts/default_ik.py`
    - `scripts/global_ik.py`
  - Import and instantiate them in the main node:
    ```python
    from scripts.default_ik import DefaultIK
    self.SCRIPT_MODES = [ForwardKin(), DefaultIK(), ...]
    ```

### 3. **Joystick Input Processing**

- The original code has complex joystick normalization, scaling, and angle limiting logic.

- **What To Do:**

  - Refactor joystick logic into helper functions or a class.
  - Migrate `getJoystickButtonStatus()` method
  - Ensure that the `ArmInputs` message type matches the ROS 2 `.msg` definition.

### 4. **Position Save/Load (JSON)**

- Uses `arm_positions.json` to read/write angle sets.

- **What To Do:**

  - Migrate file I/O logic as-is
  - Add error handling with Python 3 `with open()` and `try/except`
  - Keep JSON schema the same for compatibility

### 5. **Angle Correction Logic (**\`\`**)**

- This handles angle wrap-around to prevent joint flipping.

- **What To Do:**

  - Move this function into a utility module (e.g., `utils.py`)
  - Call it from within joystick handlers and `inverseKinematics()` loop

---

## 🧩 Should Missing Functionality Be Written Separately?

**Yes.** The best approach is to **modularize**:

| Functionality                  | Where to Place It                              |
| ------------------------------ | ---------------------------------------------- |
| Script Modes (DefaultIK, etc.) | `script_modes/` folder or submodules           |
| Service callbacks              | Inside `InverseKinematicsNode`                 |
| Joystick logic                 | As helper methods or a `JoystickHandler` class |
| IK and angle math              | `ik_library.py`, `utils.py`                    |

This structure keeps the node logic clean and each part testable.

---

## ✅ Suggested Next Steps

1. Migrate `Corrections`, `GoToArmPos`, and `SaveArmPos` services to ROS 2.
2. Port `DefaultIK`, `GlobalIK`, and `ForwardKin` classes.
3. Add launch/test files for the migrated ROS 2 nodes.
4. Validate each unit using `ros2 topic echo` and `ros2 service call`.

Let me know if you'd like me to start with any of these tasks!

