#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from auto_keyboard_msgs.msg import KeyboardGeometry, KeyboardKeyExtra  # Our custom created messages

class KeyboardSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.create_subscription(KeyboardGeometry, 'keyboard/geometry', self.geometry_callback, 10)
        self.create_subscription(KeyboardKeyExtra, 'keyboard/extra', self.extra_callback, 10)

    def geometry_callback(self, msg: KeyboardGeometry):
        self.get_logger().info(
            f"Received KeyboardGeometry: size=({msg.image_width}x{msg.image_height}), "
            f"first H val={msg.h_img_to_rect[0]:.2f}"
        )

    def extra_callback(self, msg: KeyboardKeyExtra):
        if msg.id:
            self.get_logger().info(
                f"Received KeyboardKeyExtra: keys={msg.id}, first center=({msg.center_rect_px[0].x:.1f}, {msg.center_rect_px[0].y:.1f})"
            )
        else:
            self.get_logger().info("Received KeyboardKeyExtra: no keys")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()