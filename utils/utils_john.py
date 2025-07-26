# Utility functions John is working on - will merge with Ada later

from arm_msgs.msg import ArmInputs


#########################################
# ---------------------------------------
#           Controller folder
# ---------------------------------------
#########################################

def reset_arm_inputs() -> ArmInputs:

    """
    Used in:    controller/controller/arm_controller.py             -->     Controller.__init__()
                controller/controller/arm_keyboard_controller.py    -->     KeyboardControllerNode.on_press()
                                                                    -->     KeyboardControllerNode.on_release()
    """
    
    return ArmInputs(
        l_horizontal=0,
        l_vertical=0,
        r_horizontal=0,
        r_vertical=0,
        l1=0,
        r1=0,
        l2=0,
        r2=0,
        x=0,
        o=0,
        share=0,
        options=0,
        r3=0,
        triangle=0,
        square=0,
        l3=0
    )


def map_key_to_input(key_char: str, arm_input: ArmInputs):

    """
    Used in:    controller/controller/arm_keyboard_controller.py    -->     KeyboardControllerNode.on_press()
    """

    keymap = {
        'w': ('l_vertical', 1),     # left vertical joystick emulation
        's': ('l_vertical', -1),

        'a': ('l_horizontal', 1),   # left horizontal joystick emulation
        'd': ('l_horizontal', -1),

        'i': ('r_vertical', 1),     # right vertical joystick emulation
        'k': ('r_vertical', -1),

        'j': ('r_horizontal', 1),   # right horizontal joystick emulation
        'l': ('r_horizontal', -1),

        'p': ('x', 1),              # shape button emulation
        'o': ('o', 1),
        'u': ('triangle', 1),
        ';': ('square', 1),

        'q': ('l1', 1),             # other buttons
        'e': ('r1', 1),
        'f': ('share', 1),
        'h': ('options', 1),
        'y': ('r3', 1),
    }

    if key_char in keymap:
        setattr(arm_input, *keymap[key_char])

