#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

from std_msgs.msg import Float32MultiArray


class ArmSubscriberNode(Node):
    def __init__(self, joints_callback):
        super().__init__("arm_gui_node")

        self.sub = self.create_subscription(
            Float32MultiArray,
            "arm_curr_pos",    
            self._callback,
            10,
        )

        self._joints_callback = joints_callback

    def _callback(self, msg):
        self._joints_callback(msg.data)


class SimpleWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Simple PyQt + ROS2")
        self.setGeometry(200, 200, 800, 500)

        layout = QVBoxLayout()
        self.joints_label = QLabel("Waiting for joint data...")
        layout.addWidget(self.joints_label)
        self.setLayout(layout)

    def update_joints(self, joints):
        text = ", ".join(f"{j:.2f}" for j in joints)
        self.joints_label.setText(text)


def main():
    rclpy.init()

    app = QApplication(sys.argv)

    window = SimpleWindow()

    node = ArmSubscriberNode(
        joints_callback=window.update_joints
    )

    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(10)

    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
