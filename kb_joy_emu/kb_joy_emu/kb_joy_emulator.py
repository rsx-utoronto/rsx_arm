#!/usr/bin/env python3
"""
This was ripped STRAIGHT from gemini's google suggestion.

please don't trust using this with the actual arm, just get a joystick
unless you have to, in which case good luck
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# PS4 Controller Mapping Layout (sensor_msgs/Joy format)
# Axes: [0: Left Stick L/R, 1: Left Stick U/D, 2: Right Stick L/R, 3: Left Trigger,
#        4: Right Trigger, 5: Right Stick U/D, 6: D-pad L/R, 7: D-pad U/D]
# Buttons: [0: Cross, 1: Circle, 2: Triangle, 3: Square, 4: L1, 5: R1, 6: L2, 7: R2,
#           8: Share, 9: Options, 10: PS, 11: L3, 12: R3]


class PS4Emulator(Node):
    def __init__(self):
        super().__init__('ps4_emulator')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Save terminal settings for non-blocking keyboard reads
        self.settings = termios.tcgetattr(sys.stdin)

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

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Arrow keys send 3-character escape sequences (e.g., '\x1b[A')
        # We check if data is waiting to ensure we read full escape codes
        # cleanly
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':  # Escape sequence detected
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()

        # Reset transient binary buttons on every tick so they act like taps
        self.joy_msg.buttons = [0] * 13

        # Also reset dpad
        self.joy_msg.axes[6] = 0
        self.joy_msg.axes[7] = 0

        if key == '':
            pass
        elif key == '\x03':  # Ctrl+C
            rclpy.shutdown()
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
            self.joy_msg.axes[7] = 1.0   # D-pad Up
        elif key == 'g':
            self.joy_msg.axes[7] = -1.0  # D-pad Down
        elif key == 'f':
            self.joy_msg.axes[6] = 1.0   # D-pad Left
        elif key == 'h':
            self.joy_msg.axes[6] = -1.0  # D-pad Right

        # --- BUTTONS: Action Buttons ---
        elif key == 'x':
            self.joy_msg.buttons[0] = 1  # Cross
        elif key == 'c':
            self.joy_msg.buttons[1] = 1  # Circle
        elif key == 't':
            self.joy_msg.buttons[2] = 1  # Triangle
        elif key == 'q':
            self.joy_msg.buttons[3] = 1  # Square

        # --- BUTTONS: Bumpers & Utility ---
        elif key == '1':
            self.joy_msg.buttons[4] = 1  # L1
        elif key == '2':
            self.joy_msg.buttons[5] = 1  # R1
        elif key == '3':
            self.joy_msg.buttons[6] = 1  # L2 Button
        elif key == '4':
            self.joy_msg.buttons[7] = 1  # R2 Button
        elif key == '8':
            self.joy_msg.buttons[8] = 1  # Share
        elif key == '9':
            self.joy_msg.buttons[9] = 1  # Options
        elif key == '0':
            self.joy_msg.buttons[10] = 1  # PS
        elif key == 'z':
            self.joy_msg.buttons[11] = 1  # L3
        elif key == 'm':
            self.joy_msg.buttons[12] = 1  # R3

        # --- RESET AXES ---
        elif key == ' ':
            self.joy_msg.axes = [0.0] * 8
            self.joy_msg.axes[2] = 1.0
            self.joy_msg.axes[5] = 1.0

        # Publish the state
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PS4Emulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings upon exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
