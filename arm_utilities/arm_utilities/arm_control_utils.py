from sensor_msgs.msg import Joy
from arm_msgs.msg import ArmInputs
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import numpy as np

def handle_joy_input(msg: Joy):
    arm_inputs = ArmInputs()

    # Joystick axes
    # Left joystick: axes 0 (horizontal), 1 (vertical)
    arm_inputs.l_horizontal = msg.axes[0]
    arm_inputs.l_vertical = msg.axes[1]
    # Triggers: axes 2 (left), 5 (right)
    arm_inputs.l_trigger = -0.5 * msg.axes[2] + 0.5
    arm_inputs.r_trigger = -0.5 * msg.axes[5] + 0.5
    # Right joystick: axes 3 (horizontal), 4 (vertical)
    arm_inputs.r_horizontal = msg.axes[3]
    arm_inputs.r_vertical = msg.axes[4]

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
    arm_inputs.l3 = msg.buttons[11]  # Left joystick button
    arm_inputs.r3 = msg.buttons[12]  # Right joystick button

    return arm_inputs


def handle_keyboard_input(key):
    arm_inputs = ArmInputs()  # defaults to 0.0/0

    # mapping for character keys
    char_map = {
        'w': ('l_vertical',  1.0), 's': ('l_vertical', -1.0),
        'a': ('l_horizontal',  1.0), 'd': ('l_horizontal', -1.0),
        'i': ('r_vertical',  1.0), 'k': ('r_vertical', -1.0),
        'j': ('r_horizontal', 1.0), 'l': ('r_horizontal', -1.0),
        'p': ('x', 1), 'o': ('o', 1), 'u': ('triangle', 1), ';': ('square', 1),
        'q': ('l1', 1), 'e': ('r1', 1),
        'f': ('share', 1), 'h': ('options', 1), 'y': ('r3', 1),
    }

    # mapping for special keys
    special_map = {
        keyboard.Key.space: ('r_trigger', 1.0),
        keyboard.Key.shift: ('l_trigger', 1.0),
        keyboard.Key.left:  ('dpad_left', 1),
        keyboard.Key.right: ('dpad_right', 1),
        keyboard.Key.up:    ('dpad_up', 1),
        keyboard.Key.down:  ('dpad_down', 1),
    }

    try:
        if key.char in char_map:
            attr, val = char_map[key.char]
            setattr(arm_inputs, attr, val)
    except AttributeError:
        if key in special_map:
            attr, val = special_map[key]
            setattr(arm_inputs, attr, val)

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
        target_joints[n] = manual_commands[key] * \
            speed_limits[n] + current_joints[n]

    return target_joints


def map_inputs_to_ik(arm_inputs: ArmInputs, curr_pose: Pose):
    delta = 0.05  # Incremental change for position
    delta_rot = 0.1  # Incremental change for orientation (radians)

    new_pose = Pose()
    new_pose.position.x = curr_pose.position.x
    new_pose.position.y = curr_pose.position.y + arm_inputs.l_vertical * delta
    new_pose.position.z = curr_pose.position.z + \
        (arm_inputs.r_trigger - arm_inputs.l_trigger) * delta


    x_rot = R.from_euler('xyz', [arm_inputs.r_vertical*delta_rot, 0, 0], degrees=False)
    y_rot = R.from_euler('xyz', [0, (arm_inputs.r1-arm_inputs.l1)*delta_rot, 0], degrees=False)
    z_rot = R.from_euler('xyz', [0, 0, arm_inputs.r_horizontal*delta_rot], degrees=False)
    r = R.from_quat([curr_pose.orientation.x, curr_pose.orientation.y,
                    curr_pose.orientation.z, curr_pose.orientation.w])
    
    new_r = r
    new_quat = new_r.as_quat()
    new_pose.orientation.x = new_quat[0]
    new_pose.orientation.y = new_quat[1]
    new_pose.orientation.z = new_quat[2]
    new_pose.orientation.w = new_quat[3]

    return new_pose


def clamp(value, min_value, max_value):
    """Clamps a value within a specified range."""
    return max(min_value, min(value, max_value))

#for keyboard pressing
def get_keyboard_normal(self, corners):
    corners = np.array(corners)

    # Use two edges of the keyboard plane
    v1 = corners[1] - corners[0]   
    v2 = corners[3] - corners[0]  

    # Cross product gives plane normal
    normal = np.cross(v1, v2)

    # Normalize to unit vector
    normal = normal / np.linalg.norm(normal)

    return normal


def get_intermediate_positions(self, position, corners, translation=[0,0,0]):
    #tune parameters w/ physical testing
    approach_dist = 0.1   
    press_dist = 0.01   

    #position of Tip
    position = np.array(position)

    ##Tip to End Effector translation
    translation = np.array(translation)

    normal = self.get_keyboard_normal(corners)

    # Apply translation to target position
    intermediate = position + translation

    approach = intermediate + normal * approach_dist
    press = intermediate + normal * press_dist

    return approach, press