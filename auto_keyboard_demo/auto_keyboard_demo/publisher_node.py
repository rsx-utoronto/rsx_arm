#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import time

from auto_keyboard_msgs.msg import KeyboardGeometry, KeyboardKeyExtra   # Our Custom Created Messages
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Polygon, Point32 as PolyPoint

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub_geom = self.create_publisher(KeyboardGeometry, 'keyboard/geometry', 10)
        self.pub_extra = self.create_publisher(KeyboardKeyExtra, 'keyboard/extra', 10)
        timer_period = 1.0  # publish every 1s
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # KeyboardGeometry message
        geom = KeyboardGeometry()
        geom.header = Header()
        geom.header.stamp = self.get_clock().now().to_msg()
        geom.header.frame_id = "camera_frame"

        geom.image_width = 640
        geom.image_height = 480
        geom.h_img_to_rect = [random.random() for _ in range(9)]
        geom.h_rect_to_img = [random.random() for _ in range(9)]
        geom.kb_quad_px = [
            Point32(x=random.uniform(0, 640), y=random.uniform(0, 480), z=0.0)
            for _ in range(4)
        ]
        geom.rect_frame_id = "keyboard_rectified"

        self.pub_geom.publish(geom)
        self.get_logger().info("Published KeyboardGeometry")

        # KeyboardKeyExtra message
        extra = KeyboardKeyExtra()
        extra.header = geom.header
        extra.id = ["A", "B", "C"]

        # random centers
        extra.center_rect_px = [
            Point32(x=random.uniform(0, 640), y=random.uniform(0, 480), z=0.0)
            for _ in extra.id
        ]

        # random polygons
        for _ in extra.id:
            poly = Polygon()
            poly.points = [
                PolyPoint(x=random.uniform(0, 640), y=random.uniform(0, 480), z=0.0)
                for _ in range(4)
            ]
            extra.poly_img_px.append(poly)

        extra.depth_m = [random.uniform(0.1, 1.0) for _ in extra.id]
        extra.row = [random.randint(0, 5) for _ in extra.id]
        extra.col = [random.randint(0, 10) for _ in extra.id]

        self.pub_extra.publish(extra)
        self.get_logger().info("Published KeyboardKeyExtra")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()