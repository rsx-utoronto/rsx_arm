from sensor_msgs.msg import Joy
from arm_msgs.msg import ArmInputs
from geometry_msgs.msg import Pose
# from pynput import keyboard
from scipy.spatial.transform import Rotation as R


# Axis indices (l_trigger, r_trigger, r_horizontal, r_vertical) for each joystick layout.
# Triggers rest at 1.0, so `ros2 topic echo --once /arm_joy` with the joystick untouched
# tells you which one you have.
#   standard: joy_node on native Linux, rosbags recorded that way, kb_joy_emu
#   wsl:      PS4 controller passed through to WSL with usbipd
JOY_AXIS_LAYOUTS = {
    'standard': (2, 5, 3, 4),
    'wsl': (3, 4, 2, 5),
}


def handle_joy_input(msg: Joy, layout: str = 'standard'):
    arm_inputs = ArmInputs()
    l_trigger_axis, r_trigger_axis, r_horizontal_axis, r_vertical_axis = JOY_AXIS_LAYOUTS[layout]

    # Joystick axes
    # Left joystick: axes 0 (horizontal), 1 (vertical)
    arm_inputs.l_horizontal = msg.axes[0]
    arm_inputs.l_vertical = msg.axes[1]

    # Triggers: 1.0 released, -1.0 fully pressed -> mapped to 0..1
    arm_inputs.l_trigger = -0.5 * msg.axes[l_trigger_axis] + 0.5
    arm_inputs.r_trigger = -0.5 * msg.axes[r_trigger_axis] + 0.5

    # Right joystick
    arm_inputs.r_horizontal = msg.axes[r_horizontal_axis]
    arm_inputs.r_vertical = msg.axes[r_vertical_axis]

    # D-Pad: axes 6 (horizontal), 7 (vertical)
    arm_inputs.dpad_left = 1 if msg.axes[6] == -1 else 0
    arm_inputs.dpad_right = 1 if msg.axes[6] == 1 else 0
    arm_inputs.dpad_up = 1 if msg.axes[7] == 1 else 0
    arm_inputs.dpad_down = 1 if msg.axes[7] == -1 else 0

    # Joystick buttons
    arm_inputs.l1 = msg.buttons[4]  # L1 button
    arm_inputs.r1 = msg.buttons[5]  # R1 button
    arm_inputs.x = msg.buttons[0]  # X button
    arm_inputs.circle = msg.buttons[1]  # Circle button
    arm_inputs.triangle = msg.buttons[2]  # Triangle button
    arm_inputs.square = msg.buttons[3]  # Square button
    arm_inputs.share = msg.buttons[8]  # Share button
    arm_inputs.options = msg.buttons[9]  # Options button
    # arm_inputs.l3 = msg.buttons[11]  # Left joystick button
    # arm_inputs.r3 = msg.buttons[12]  # Right joystick button

    return arm_inputs


def map_inputs_to_manual(arm_inputs: ArmInputs, speed_limits: list, current_joints: list):
    manual_commands = {
        'base_rotation': arm_inputs.l_horizontal,
        'shoulder': arm_inputs.l_vertical,
        'elbow': arm_inputs.r_vertical,
        'elbow_roll': arm_inputs.r_horizontal,
        'wrist_pitch': arm_inputs.r1 - arm_inputs.l1,
        # R1 for clockwise, L1 for counterclockwise
        'wrist_roll': arm_inputs.r_trigger - arm_inputs.l_trigger,
        'gripper': arm_inputs.x - arm_inputs.circle  # R2 to open, L2 to close
    }
    target_joints = current_joints
    for n, key in enumerate(manual_commands.keys()):
        target_joints[n] = manual_commands[key] *\
            speed_limits[n] + current_joints[n]

    return target_joints


def map_inputs_to_ik(arm_inputs: ArmInputs, curr_pose: Pose):
    delta = 0.005  # Incremental change for position
    delta_rot = 0.02  # Incremental change for orientation (radians)

    new_pose = Pose()
    new_pose.position.x = curr_pose.position.x + arm_inputs.l_horizontal * delta
    new_pose.position.y = curr_pose.position.y + arm_inputs.l_vertical * delta
    new_pose.position.z = curr_pose.position.z +\
        (arm_inputs.r_trigger - arm_inputs.l_trigger) * delta

    delta_r = R.from_euler('XYZ', [
        arm_inputs.r_vertical * delta_rot,
        (arm_inputs.r1 - arm_inputs.l1) * delta_rot,
        arm_inputs.r_horizontal * delta_rot
    ], degrees=False)
    r = R.from_quat([curr_pose.orientation.x, curr_pose.orientation.y,
                    curr_pose.orientation.z, curr_pose.orientation.w])

    # TODO: needs testing
    new_r = delta_r * r  # Apply incremental rotation to current orientation
    new_quat = new_r.as_quat()
    new_pose.orientation.x = new_quat[0]
    new_pose.orientation.y = new_quat[1]
    new_pose.orientation.z = new_quat[2]
    new_pose.orientation.w = new_quat[3]

    return new_pose


def clamp(value, min_value, max_value):
    """Clamps a value within a specified range."""
    return max(min_value, min(value, max_value))
