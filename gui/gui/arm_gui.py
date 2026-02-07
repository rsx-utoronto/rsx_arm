#!/usr/bin/env python3
import sys
import time
import math

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

from std_msgs.msg import String, Float32MultiArray

from arm_utilities.arm_enum_utils import ArmState, SafetyErrors

class Window(Node, QWidget):
    def __init__(self, stale_after_s=1.0):
        Node.__init__(self, "arm_gui_node")  
        QWidget.__init__(self)

        self.state = ArmState.IDLE
        self.curr_joints = []
        self.target_joints = []

        self.last_update_time = {current: 0.0 for current in ["state", "curr", "target"]}         

        self.stale_after_s = float(stale_after_s)

        self.setWindowTitle("Arm Display GUI")
        self.setGeometry(200, 200, 900, 250)

        layout = QVBoxLayout(self)
        self.state_label = QLabel("State: (waiting...)")
        self.curr_label = QLabel("Current joints: (waiting...)")
        self.target_label = QLabel("Target joints: (waiting...)")
        layout.addWidget(self.state_label)
        layout.addWidget(self.curr_label)
        layout.addWidget(self.target_label)

 
        self.create_subscription(String, "arm_state", self.on_state, 10)
        self.create_subscription(
            Float32MultiArray, "arm_curr_angles", self.on_curr, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "safe_arm_target_joints",
            self.on_target,
            10,
        )



        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._mark_stale_if_needed)
        self.watchdog.start(200)

    def on_state(self, msg: String):
        self.state = ArmState(msg.data)
        self.last_update_time["state"] = time.time()
        self.state_label.setText(f"State: {msg.data}")

    def on_curr(self, msg: Float32MultiArray):
        self.curr_joints = list(msg.data)
        self.last_update_time["curr"] = time.time()
        self.curr_label.setText(
            f"Current joints: {self._fmt_joints(list(msg.data))}"
        )

    def on_target(self, msg: Float32MultiArray):
        self.target_joints = list(msg.data)
        self.last_update_time["target"] = time.time()   
        self.target_label.setText(
            f"Target joints: {self._fmt_joints(list(msg.data))}"
        )

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

    def _mark_stale_if_needed(self):
        now = time.time()
        t = self.stale_after_s

        if now - self.last_update_time["state"] > t:
            self.state_label.setText("State: (waiting...)")
        if now - self.last_update_time["curr"] > t:
            self.curr_label.setText("Current joints: (waiting...)")
        if now - self.last_update_time["target"] > t:
            self.target_label.setText("Target joints: (waiting...)")


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    win = Window(stale_after_s=1.0)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(win, timeout_sec=0.01))
    ros_timer.start(10)

    win.show()

    try:
        code = app.exec()
    finally:
    
        win.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == "__main__":
    main()
