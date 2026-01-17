#!/usr/bin/env python3
import sys
import time
import math

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

from std_msgs.msg import String, Float32MultiArray


class ArmGuiSubscriber(Node):
    def __init__(self, on_state, on_curr, on_target):
        super().__init__("arm_gui_node")

        self.create_subscription(String, "arm_state", lambda m: on_state(m.data), 10)
        self.create_subscription(
            Float32MultiArray, "arm_curr_angles", lambda m: on_curr(list(m.data)), 10
        )
        self.create_subscription(
            Float32MultiArray,
            "safe_arm_target_joints",
            lambda m: on_target(list(m.data)),
            10,
        )


class Window(QWidget):
    state_sig = pyqtSignal(str)
    curr_sig = pyqtSignal(list)
    target_sig = pyqtSignal(list)

    def __init__(self, stale_after_s=1.0):
        super().__init__()

        self.stale_after_s = float(stale_after_s)
        self._last = {"state": 0.0, "curr": 0.0, "target": 0.0}

        self.setWindowTitle("Arm GUI")
        self.setGeometry(200, 200, 900, 250)

        layout = QVBoxLayout(self)
        self.state_label = QLabel("State: (waiting...)")
        self.curr_label = QLabel("Current joints: (waiting...)")
        self.target_label = QLabel("Target joints: (waiting...)")
        layout.addWidget(self.state_label)
        layout.addWidget(self.curr_label)
        layout.addWidget(self.target_label)

        self.state_sig.connect(self._set_state)
        self.curr_sig.connect(self._set_curr)
        self.target_sig.connect(self._set_target)

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._mark_stale_if_needed)
        self.watchdog.start(200)

    def on_state(self, state: str):
        self._last["state"] = time.time()
        self.state_sig.emit(state)

    def on_curr(self, joints: list):
        self._last["curr"] = time.time()
        self.curr_sig.emit(joints)

    def on_target(self, joints: list):
        self._last["target"] = time.time()
        self.target_sig.emit(joints)

    def _set_state(self, state: str):
        self.state_label.setText(f"State: {state}")

    def _fmt_joints(self, joints: list) -> str:
        if not joints:
            return "(no data)"
        parts = []
        for i, j in enumerate(joints, start=1):
            try:
                jf = float(j)
                parts.append(f"J{i}={jf:.2f}" if math.isfinite(jf) else f"J{i}=?")
            except Exception:
                parts.append(f"J{i}=?")
        return "  |  ".join(parts)

    def _set_curr(self, joints: list):
        self.curr_label.setText(f"Current joints: {self._fmt_joints(joints)}")

    def _set_target(self, joints: list):
        self.target_label.setText(f"Target joints: {self._fmt_joints(joints)}")

    def _mark_stale_if_needed(self):
        now = time.time()
        t = self.stale_after_s

        if now - self._last["state"] > t:
            self.state_label.setText("State: (waiting...)")
        if now - self._last["curr"] > t:
            self.curr_label.setText("Current joints: (waiting...)")
        if now - self._last["target"] > t:
            self.target_label.setText("Target joints: (waiting...)")


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    win = Window(stale_after_s=1.0)
    node = ArmGuiSubscriber(win.on_state, win.on_curr, win.on_target)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)

    win.show()

    try:
        code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == "__main__":
    main()
