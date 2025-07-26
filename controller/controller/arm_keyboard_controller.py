#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from arm_msgs.msg import ArmInputs
from pynput import keyboard
 
from utils.arm_input_utils import reset_arm_inputs

class KeyboardControllerNode(Node):
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

        keyboardToController = reset_arm_inputs()

       
        try:
            map_key_to_input(key.char, keyboardToController)

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
        keyboardToController = reset_arm_inputs()

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
