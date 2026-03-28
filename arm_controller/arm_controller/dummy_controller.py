#!/usr/bin/env python3

'''
    Literally just sends a joy message periodically so the main_controller thinks
    that we have an actual controller plugged in and updates its state.
'''

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class dummy_controller(Node):
    def __init__(self):
        super().__init__("Dummy_Controller")
        
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

        # 0.1s timer to publish the message out
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create the message
        msg = Joy()

        # 1. Update the timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()

        # 2. Fill axes and buttons with zeros
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 13

        # For path planning mode
        msg.axes[6] = 1

        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dummy_ctrl = dummy_controller()

    try:
        rclpy.spin(dummy_ctrl)
    except KeyboardInterrupt:
        dummy_ctrl.get_logger().info("Keyboard interrupt received")
    finally:
        dummy_ctrl.destroy_node()
        rclpy.shutdown()