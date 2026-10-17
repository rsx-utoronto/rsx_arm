#!/usr/bin/env python3
"""
This was ripped STRAIGHT from gemini's google suggestion.

please don't trust using this with the actual arm, just get a joystick
unless you have to, in which case good luck
"""

import os
import sys
import select
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

# PS4 Controller Mapping Layout (sensor_msgs/Joy format)
# Axes: [0: Left Stick L/R, 1: Left Stick U/D, 2: Right Stick L/R, 3: Left Trigger,
#        4: Right Trigger, 5: Right Stick U/D, 6: D-pad L/R, 7: D-pad U/D]
# Buttons: [0: Cross, 1: Circle, 2: Triangle, 3: Square, 4: L1, 5: R1, 6: L2, 7: R2,
#           8: Share, 9: Options, 10: PS, 11: L3, 12: R3]

# Taps (buttons, D-pad) are held for this many ticks so a downstream
# throttle (e.g. 10 Hz with throttle_on:=true) can't drop them
TAP_HOLD_TICKS = 3


class PS4Emulator(Node):
    def __init__(self):
        super().__init__('ps4_emulator')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Save terminal settings, then switch to cbreak once (unbuffered key
        # reads, Ctrl+C still works) instead of toggling raw mode every tick
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Remaining ticks each tap is held for
        self.button_ticks = [0] * 13
        self.dpad = [0.0, 0.0]        # axes 6, 7
        self.dpad_ticks = [0, 0]

        # Initialize standard PS4 Joy message structure
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8       # 8 standard analog axes
        self.joy_msg.buttons = [0] * 13     # 13 standard binary buttons

        # Set trigger axes to 1.0 by default (unpressed state for PS4
        # controllers in ROS)
        self.joy_msg.axes[2] = 1.0  # L2 Analog
        self.joy_msg.axes[5] = 1.0  # R2 Analog

        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("PS4 Controller Emulator Initialized!")
        print("\n=== CONTROLS ===")
        print("Left Stick (Axes 0, 1)  : W/S (Up/Down), A/D (Left/Right)")
        print("Right Stick (Axes 3, 4) : I/K (Up/Down), J/L (Left/Right)")
        print("D-Pad (Axes 6, 7)       : T/G (Up/Down), F/H (Left/Right)")
        print("Triggers (Axes 2, 5)    : E (L2 Analog), O (R2 Analog)")
        print("Buttons (0-3)           : X=Cross, C=Circle, V=Triangle, Q=Square")
        print("Bumpers & Triggers (4-7): 1=L1, 2=R1, 3=L2_Btn, 4=R2_Btn")
        print("System (8-10)           : 8=Share, 9=Options, 0=PS")
        print("Stick Clicks (11-12)    : Z=L3, M=R3")
        print("Reset Axes              : Spacebar")
        print("Exit                    : Ctrl+C\n")

    def get_keys(self):
        # Drain everything typed since the last tick without blocking, so held
        # keys (auto-repeat) don't back up and keep acting after release.
        # os.read bypasses Python's stdin buffer, which select() can't see.
        keys = ''
        fd = sys.stdin.fileno()
        while select.select([fd], [], [], 0)[0]:
            data = os.read(fd, 1024)
            if not data:
                break
            keys += data.decode(errors='ignore')
        return keys

    def tap_button(self, index):
        self.button_ticks[index] = TAP_HOLD_TICKS

    def tap_dpad(self, axis, value):
        self.dpad[axis] = value
        self.dpad_ticks[axis] = TAP_HOLD_TICKS

    def timer_callback(self):
        for key in self.get_keys():
            self.handle_key(key)

        # Taps stay pressed for TAP_HOLD_TICKS ticks, then release
        self.joy_msg.buttons = [1 if t > 0 else 0 for t in self.button_ticks]
        self.button_ticks = [max(0, t - 1) for t in self.button_ticks]
        for axis in range(2):
            self.joy_msg.axes[6 + axis] = self.dpad[axis] if self.dpad_ticks[axis] > 0 else 0.0
            self.dpad_ticks[axis] = max(0, self.dpad_ticks[axis] - 1)

        # Publish the state
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)

    def handle_key(self, key):
        if key == '\x03':  # Ctrl+C
            rclpy.try_shutdown()
            sys.exit(0)

        # --- AXES: Left Stick (W/A/S/D) ---
        elif key == 'w':
            self.joy_msg.axes[1] = min(1.0, self.joy_msg.axes[1] + 0.2)
        elif key == 's':
            self.joy_msg.axes[1] = max(-1.0, self.joy_msg.axes[1] - 0.2)
        elif key == 'a':
            self.joy_msg.axes[0] = min(1.0, self.joy_msg.axes[0] + 0.2)
        elif key == 'd':
            self.joy_msg.axes[0] = max(-1.0, self.joy_msg.axes[0] - 0.2)

        # --- AXES: Right Stick (I/J/K/L) ---
        elif key == 'i':
            self.joy_msg.axes[4] = min(1.0, self.joy_msg.axes[4] + 0.2)
        elif key == 'k':
            self.joy_msg.axes[4] = max(-1.0, self.joy_msg.axes[4] - 0.2)
        elif key == 'j':
            self.joy_msg.axes[3] = min(1.0, self.joy_msg.axes[3] + 0.2)
        elif key == 'l':
            self.joy_msg.axes[3] = max(-1.0, self.joy_msg.axes[3] - 0.2)

        # --- AXES: Analog Triggers (E/O) toggle down to -1.0 ---
        elif key == 'e':
            self.joy_msg.axes[2] = -1.0 if self.joy_msg.axes[2] == 1.0 else 1.0
        elif key == 'o':
            self.joy_msg.axes[5] = -1.0 if self.joy_msg.axes[5] == 1.0 else 1.0

        # --- AXES: D-Pad (Rebound to single character letters) ---
        elif key == 't':
            self.tap_dpad(1, 1.0)   # D-pad Up
        elif key == 'g':
            self.tap_dpad(1, -1.0)  # D-pad Down
        elif key == 'f':
            self.tap_dpad(0, 1.0)   # D-pad Left
        elif key == 'h':
            self.tap_dpad(0, -1.0)  # D-pad Right

        # --- BUTTONS: Action Buttons ---
        elif key == 'x':
            self.tap_button(0)  # Cross
        elif key == 'c':
            self.tap_button(1)  # Circle
        elif key == 'v':
            self.tap_button(2)  # Triangle
        elif key == 'q':
            self.tap_button(3)  # Square

        # --- BUTTONS: Bumpers & Utility ---
        elif key == '1':
            self.tap_button(4)  # L1
        elif key == '2':
            self.tap_button(5)  # R1
        elif key == '3':
            self.tap_button(6)  # L2 Button
        elif key == '4':
            self.tap_button(7)  # R2 Button
        elif key == '8':
            self.tap_button(8)  # Share
        elif key == '9':
            self.tap_button(9)  # Options
        elif key == '0':
            self.tap_button(10)  # PS
        elif key == 'z':
            self.tap_button(11)  # L3
        elif key == 'm':
            self.tap_button(12)  # R3

        # --- RESET AXES ---
        elif key == ' ':
            self.joy_msg.axes = [0.0] * 8
            self.joy_msg.axes[2] = 1.0
            self.joy_msg.axes[5] = 1.0


def main(args=None):
    rclpy.init(args=args)
    node = PS4Emulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Restore terminal settings upon exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
