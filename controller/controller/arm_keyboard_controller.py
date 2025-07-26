#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rover.msg import ArmInputs
from pynput import keyboard

class KeyboardControllerNode():
    def __init__(self):
        super().__init__('arm_keyboard_controller')

        self.inputPublisher = self.create_publisher(ArmInputs, 'arm_inputs', 10)
        self.statePublisher = self.create_publisher(String, 'arm_state', 10)

        # Start keyboard listener in a non-blocking way (starting a new thread)
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
   

    def on_press(self, key):
        keyboardToController = ArmInputs()

        keyboardToController.l_horizontal = 0
        keyboardToController.l_vertical   = 0
        keyboardToController.r_horizontal = 0
        keyboardToController.r_vertical   = 0
        keyboardToController.l1           = 0
        keyboardToController.r1           = 0
        keyboardToController.l2           = 0
        keyboardToController.r2           = 0
        keyboardToController.x            = 0
        keyboardToController.o            = 0
        keyboardToController.share        = 0
        keyboardToController.options      = 0
        keyboardToController.r3    = 0

       
        try:
            # left vertical joystick emulation
            if key.char == 'w':
                keyboardToController.l_vertical = 1
            elif key.char == "s":
                keyboardToController.l_vertical = -1

            # left horizontal joystick emulation
            if key.char == "a":
                keyboardToController.l_horizontal = 1
            elif key.char == "d":
                keyboardToController.l_horizontal = -1

            # right vertical joystick emulation
            if key.char == 'i':
                keyboardToController.r_vertical = 1
            elif key.char == "k":
                keyboardToController.r_vertical = -1

            # right horizontal joystick emulation
            if key.char == "j":
                keyboardToController.r_horizontal = 1
            elif key.char == "l":
                keyboardToController.r_horizontal = -1

            # shape button emulation
            if key.char == "p":
                keyboardToController.x = 1
            if key.char == "o":
                keyboardToController.o = 1
            if key.char == "u":
                keyboardToController.triangle = 1
            if key.char == ";":
                keyboardToController.square = 1

            # other buttons
            if key.char == "q":
                keyboardToController.l1 = 1
            if key.char == "e":
                keyboardToController.r1 = 1
            if key.char == "f":
                keyboardToController.share = 1
            if key.char == "h":
                keyboardToController.options = 1
            if key.char == "y":
                keyboardToController.r3 = 1
        except AttributeError:
            # left and right triggers
            if key == keyboard.Key.space:
                keyboardToController.r2 = 1
            if key == keyboard.Key.shift:
                keyboardToController.l2 = 1

            # emulate d-pad as arrow keys
            if key == keyboard.Key.left:
                self.statePubliser.publish("Manual")
            if key == keyboard.Key.right:
                self.statePubliser.publish("IK")
            if key == keyboard.Key.up:
                self.statePubliser.publish("Setup")
            if key == keyboard.Key.down:
                self.statePubliser.publish("Idle")

        self.inputPublisher.publish(keyboardToController)


    def on_release(self, key):
        # Reset all values to 0 on release
        keyboardToController = ArmInputs()

        keyboardToController.l_horizontal = 0
        keyboardToController.l_vertical   = 0
        keyboardToController.r_horizontal = 0
        keyboardToController.r_vertical   = 0
        keyboardToController.l1           = 0
        keyboardToController.r1           = 0
        keyboardToController.l2           = 0
        keyboardToController.r2           = 0
        keyboardToController.x            = 0
        keyboardToController.o            = 0
        keyboardToController.share        = 0
        keyboardToController.options      = 0
        keyboardToController.r3    = 0

        self.inputPublisher.publish(keyboardToController)

        if key == keyboard.Key.esc:
            # Stop the listener and shut down node
            self.listener.stop()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking spin to allow keyboard thread
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
