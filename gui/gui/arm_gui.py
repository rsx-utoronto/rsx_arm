#!/usr/bin/env python3
"""
Cyberpunk-themed ROS2 GUI for robotic arm monitoring
Supports custom TargetPositionArray message type
"""
import sys
import time
import math
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np

from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QFrame, QScrollArea, QGridLayout, QCheckBox,
    QGroupBox
)
from PyQt5.QtGui import QFont, QColor, QPalette, QImage, QPixmap

from std_msgs.msg import String, Float32MultiArray, Bool
from sensor_msgs.msg import Image

from arm_msgs.msg import TargetPositionArray, TargetPosition, ArmStatuses  # Custom message type for target positions of keyboard

from arm_utilities.arm_enum_utils import ArmState, SafetyErrors


class CyberpunkLabel(QLabel):
    """Custom label with cyberpunk styling"""
    def __init__(self, text="", glow_color="#00ff00"):
        super().__init__(text)
        self.glow_color = glow_color
        self.apply_style()
    
    def apply_style(self):
        self.setStyleSheet(f"""
            QLabel {{
                color: {self.glow_color};
                font-family: 'Courier New', monospace;
                font-size: 11pt;
                padding: 5px;
                # text-shadow: 0 0 10px {self.glow_color};
            }}
        """)


class StatusIndicator(QFrame):
    """Cyberpunk-style status indicator with glow effect"""
    def __init__(self, title: str, color: str = "#00ff00"):
        super().__init__()
        self.title = title
        self.color = color
        self.status_text = "(waiting...)"
        
        self.setFrameStyle(QFrame.Box)
        self.setStyleSheet(f"""
            QFrame {{
                border: 2px solid {color};
                border-radius: 5px;
                background-color: rgba(0, 0, 0, 180);
                margin: 3px;
            }}
        """)
        
        layout = QVBoxLayout(self)
        
        # Title
        self.title_label = QLabel(f"▸ {title}")
        self.title_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                font-family: 'Courier New', monospace;
                font-size: 10pt;
                font-weight: bold;
                border: none;
            }}
        """)
        
        # Status
        self.status_label = QLabel(self.status_text)
        self.status_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                font-family: 'Courier New', monospace;
                font-size: 11pt;
                border: none;
                padding: 5px;
            }}
        """)
        self.status_label.setWordWrap(True)
        
        layout.addWidget(self.title_label)
        layout.addWidget(self.status_label)
    
    def update_status(self, text: str):
        self.status_text = text
        self.status_label.setText(text)
    
    def set_stale(self):
        self.status_label.setText("(waiting...)")


class TargetDisplay(QFrame):
    """Display for individual target with distance information"""
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        
        self.setFrameStyle(QFrame.Box)
        self.setStyleSheet("""
            QFrame {
                border: 1px solid #00ddff;
                border-radius: 3px;
                background-color: rgba(0, 20, 30, 200);
                margin: 2px;
                padding: 5px;
            }
        """)
        
        layout = QGridLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(8, 5, 8, 5)
        
        # Target name
        name_label = QLabel(f"◆ {name}")
        name_label.setStyleSheet("""
            QLabel {
                color: #00ddff;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
                font-weight: bold;
                border: none;
            }
        """)
        layout.addWidget(name_label, 0, 0, 1, 4)
        
        # Distance labels
        self.dist_label = QLabel("Dist: ---")
        self.x_label = QLabel("X: ---")
        self.y_label = QLabel("Y: ---")
        self.z_label = QLabel("Z: ---")
        
        for lbl in [self.dist_label, self.x_label, self.y_label, self.z_label]:
            lbl.setStyleSheet("""
                QLabel {
                    color: #00ffaa;
                    font-family: 'Courier New', monospace;
                    font-size: 9pt;
                    border: none;
                }
            """)
        
        layout.addWidget(self.dist_label, 1, 0)
        layout.addWidget(self.x_label, 1, 1)
        layout.addWidget(self.y_label, 1, 2)
        layout.addWidget(self.z_label, 1, 3)
    
    def update_data(self, x: float, y: float, z: float, distance: float):
        self.dist_label.setText(f"Dist: {distance:.3f}m")
        self.x_label.setText(f"X: {x:.3f}m")
        self.y_label.setText(f"Y: {y:.3f}m")
        self.z_label.setText(f"Z: {z:.3f}m")


class CameraFeed(QLabel):
    """Camera feed display with cyberpunk border"""
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 480)
        self.setMaximumSize(800, 600)
        self.setScaledContents(True)
        self.setStyleSheet("""
            QLabel {
                border: 3px solid #ff00ff;
                background-color: #000000;
                border-radius: 5px;
            }
        """)
        self.setText("CAMERA FEED\n(waiting...)")
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont('Courier New', 14, QFont.Bold))
        
        # Set text color
        palette = self.palette()
        palette.setColor(QPalette.WindowText, QColor("#ff00ff"))
        self.setPalette(palette)
    
    def update_image(self, cv_image):
        """Convert OpenCV image to QPixmap and display"""
        try:
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            # Add cyberpunk color overlay (slight green/cyan tint)
            overlay = cv_image.copy()
            overlay[:, :, 1] = np.clip(overlay[:, :, 1] * 1.1, 0, 255)  # Boost green
            overlay[:, :, 0] = np.clip(overlay[:, :, 0] * 0.9, 0, 255)  # Reduce blue
            
            q_img = QImage(overlay.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            self.setPixmap(pixmap)
        except Exception as e:
            print(f"Error updating camera feed: {e}")


class ArmGUI(Node, QWidget):
    """Main GUI class for arm monitoring interface"""
    
    # Signals for thread-safe GUI updates
    image_signal = pyqtSignal(object)
    
    def __init__(self, stale_after_s=1.0):
        Node.__init__(self, "arm_gui_node")  
        QWidget.__init__(self)
        
        # ROS data storage
        self.state = ArmState.IDLE
        self.curr_joints = []
        self.target_joints = []
        self.curr_pose = []
        self.limit_switches = {}
        self.safety_status = ""
        self.homing_status = ""
        self.path_planning_status = ""
        self.targets: Dict[str, tuple] = {}  # name -> (x, y, z, distance)
        
        # Track last update times for staleness detection
        self.last_update_time = {
            "state": 0.0,
            "curr_joints": 0.0,
            "target_joints": 0.0,
            "curr_pose": 0.0,
            "camera": 0.0,
            "targets": 0.0,
            "limit_switches": 0.0,
            "safety": 0.0,
            "homing": 0.0,
            "path_planning": 0.0
        }
        
        self.stale_after_s = float(stale_after_s)
        self.bridge = CvBridge()
        
        # Display toggles for target information
        self.show_distance = True
        self.show_x = True
        self.show_y = True
        self.show_z = True
        
        # Setup GUI and ROS
        self.setup_ui()
        self.setup_ros_subscriptions()
        
        # Connect signals
        self.image_signal.connect(self._update_camera_display)
        
        # Watchdog timer for staleness checking
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._mark_stale_if_needed)
        self.watchdog.start(200)
        self.target_displays = {}
    
    def setup_ui(self):
        """Setup the main UI with cyberpunk theme"""
        self.setWindowTitle("◢ ARM CONTROL INTERFACE ◣")
        self.setGeometry(100, 100, 1400, 900)
        
        # Main dark background with neon accents
        self.setStyleSheet("""
            QWidget {
                background-color: #0a0a0a;
                color: #00ff00;
            }
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QCheckBox {
                color: #00ffff;
                font-family: 'Courier New', monospace;
                font-size: 9pt;
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 15px;
                height: 15px;
                border: 2px solid #00ffff;
                background-color: #000000;
                border-radius: 2px;
            }
            QCheckBox::indicator:checked {
                background-color: #00ffff;
            }
            QCheckBox::indicator:hover {
                border-color: #00ffff;
            }
        """)
        
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(10)
        
        # ========== LEFT PANEL - Camera and Targets ==========
        left_panel = QVBoxLayout()
        
        # Camera feed section
        camera_group = self.create_group_box("▸ VISUAL FEED", "#ff00ff")
        camera_layout = QVBoxLayout()
        self.camera_feed = CameraFeed()
        camera_layout.addWidget(self.camera_feed)
        camera_group.setLayout(camera_layout)
        left_panel.addWidget(camera_group)
        
        # Targets tracking section
        targets_group = self.create_group_box("▸ TARGET TRACKING", "#00ddff")
        targets_layout = QVBoxLayout()
        
        # Toggle controls for target display
        toggle_layout = QHBoxLayout()
        self.dist_check = QCheckBox("Distance")
        self.x_check = QCheckBox("X-Axis")
        self.y_check = QCheckBox("Y-Axis")
        self.z_check = QCheckBox("Z-Axis")
        
        for cb in [self.dist_check, self.x_check, self.y_check, self.z_check]:
            cb.setChecked(True)
            toggle_layout.addWidget(cb)
        
        # Connect toggle signals
        self.dist_check.stateChanged.connect(lambda: self.toggle_target_display())
        self.x_check.stateChanged.connect(lambda: self.toggle_target_display())
        self.y_check.stateChanged.connect(lambda: self.toggle_target_display())
        self.z_check.stateChanged.connect(lambda: self.toggle_target_display())
        
        targets_layout.addLayout(toggle_layout)
        
        # Scrollable target list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(200)
        
        self.targets_widget = QWidget()
        self.targets_layout = QVBoxLayout(self.targets_widget)
        self.targets_layout.addStretch()
        
        scroll.setWidget(self.targets_widget)
        targets_layout.addWidget(scroll)
        
        targets_group.setLayout(targets_layout)
        left_panel.addWidget(targets_group)
        
        main_layout.addLayout(left_panel, 2)
        
        # ========== RIGHT PANEL - Status Information ==========
        right_panel = QVBoxLayout()
        
        # System status section
        status_group = self.create_group_box("▸ SYSTEM STATUS", "#00ff00")
        status_layout = QVBoxLayout()
        
        self.state_indicator = StatusIndicator("ARM STATE", "#00ff00")
        self.safety_indicator = StatusIndicator("SAFETY STATUS", "#ffaa00")
        self.homing_indicator = StatusIndicator("HOMING STATUS", "#00aaff")
        self.path_indicator = StatusIndicator("PATH PLANNING", "#ff00aa")
        
        status_layout.addWidget(self.state_indicator)
        status_layout.addWidget(self.safety_indicator)
        status_layout.addWidget(self.homing_indicator)
        status_layout.addWidget(self.path_indicator)
        
        status_group.setLayout(status_layout)
        right_panel.addWidget(status_group)
        
        # Joint angles section
        joints_group = self.create_group_box("▸ JOINT CONFIGURATION", "#ffff00")
        joints_layout = QVBoxLayout()
        
        self.curr_joints_indicator = StatusIndicator("CURRENT ANGLES", "#00ff00")
        self.target_joints_indicator = StatusIndicator("TARGET ANGLES", "#ffaa00")
        
        joints_layout.addWidget(self.curr_joints_indicator)
        joints_layout.addWidget(self.target_joints_indicator)
        
        joints_group.setLayout(joints_layout)
        right_panel.addWidget(joints_group)
        
        # Pose section
        pose_group = self.create_group_box("▸ END EFFECTOR POSE", "#ff6600")
        pose_layout = QVBoxLayout()
        
        self.pose_indicator = StatusIndicator("CURRENT POSE", "#ff6600")
        
        pose_layout.addWidget(self.pose_indicator)
        pose_group.setLayout(pose_layout)
        right_panel.addWidget(pose_group)
        
        # Limit switches section
        limits_group = self.create_group_box("▸ LIMIT SWITCHES", "#ff0066")
        limits_layout = QVBoxLayout()
        
        self.limit_switches_indicator = StatusIndicator("SWITCH STATUS", "#ff0066")
        
        limits_layout.addWidget(self.limit_switches_indicator)
        limits_group.setLayout(limits_layout)
        right_panel.addWidget(limits_group)
        
        right_panel.addStretch()
        
        main_layout.addLayout(right_panel, 1)
    
    def create_group_box(self, title: str, color: str) -> QGroupBox:
        """Create a styled group box with cyberpunk aesthetics"""
        group = QGroupBox(title)
        group.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {color};
                border-radius: 8px;
                margin-top: 10px;
                font-family: 'Courier New', monospace;
                font-size: 12pt;
                font-weight: bold;
                color: {color};
                padding-top: 15px;
                background-color: rgba(0, 0, 0, 100);
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 5px 10px;
                background-color: #0a0a0a;
            }}
        """)
        return group
    
    def setup_ros_subscriptions(self):
        """
        Setup ROS2 topic subscriptions
        TODO: Update topic names to match your system
        """
        # Existing subscriptions from original code
        self.create_subscription(String, "arm_state", self.on_state, 10)
        self.create_subscription(Float32MultiArray, "arm_curr_angles", self.on_curr_joints, 10)
        self.create_subscription(Float32MultiArray, "safe_arm_target_joints", self.on_target_joints, 10)
        
        # New subscriptions - UPDATE TOPIC NAMES AS NEEDED
        # Camera feed
        self.create_subscription(Image, "/annotated_video", self.on_camera, 10)
        self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.on_camera, 10)  # Optional raw feed
        
        # Current pose (x, y, z, roll, pitch, yaw)
        self.create_subscription(Float32MultiArray, "/arm_curr_pose", self.on_curr_pose, 10)
        
        self.create_subscription(TargetPosition, "/key_targets", self.on_targets, 10)
        
        # Limit switches status
        self.create_subscription(ArmStatuses, "/arm_lim_switches", self.on_limit_switches, 10)
        
        # Safety status
        self.create_subscription(ArmStatuses, "/arm_safety_status", self.on_safety_status, 10)
        
        # Homing status
        self.create_subscription(String, "/arm_homing_status", self.on_homing_status, 10)
        
        # Path planning execution status
        self.create_subscription(String, "/arm_path_planning_status", self.on_path_planning_status, 10)
    
    # ========== ROS Callback Methods ==========
    
    def on_state(self, msg: String):
        """Handle arm state updates"""
        self.state = ArmState[msg.data]
        self.last_update_time["state"] = time.time()
        self.state_indicator.update_status(msg.data)
    
    def on_curr_joints(self, msg: Float32MultiArray):
        """Handle current joint angles update"""
        self.curr_joints = list(msg.data)
        self.last_update_time["curr_joints"] = time.time()
        self.curr_joints_indicator.update_status(self._fmt_joints(list(msg.data)))
    
    def on_target_joints(self, msg: Float32MultiArray):
        """Handle target joint angles update"""
        self.target_joints = list(msg.data)
        self.last_update_time["target_joints"] = time.time()
        self.target_joints_indicator.update_status(self._fmt_joints(list(msg.data)))
    
    def on_curr_pose(self, msg: Float32MultiArray):
        """
        Handle current pose update (x, y, z, roll, pitch, yaw)
        Expected array: [x, y, z, roll, pitch, yaw]
        """
        self.curr_pose = list(msg.data)
        self.last_update_time["curr_pose"] = time.time()
        
        if len(self.curr_pose) >= 6:
            pose_str = f"X:{self.curr_pose[0]:.3f} Y:{self.curr_pose[1]:.3f} Z:{self.curr_pose[2]:.3f}\n"
            pose_str += f"R:{math.degrees(self.curr_pose[3]):.1f}° P:{math.degrees(self.curr_pose[4]):.1f}° Y:{math.degrees(self.curr_pose[5]):.1f}°"
            self.pose_indicator.update_status(pose_str)
        elif len(self.curr_pose) >= 3:
            pose_str = f"X:{self.curr_pose[0]:.3f} Y:{self.curr_pose[1]:.3f} Z:{self.curr_pose[2]:.3f}"
            self.pose_indicator.update_status(pose_str)
    
    def on_camera(self, msg: Image):
        """
        Handle camera image messages
        Converts ROS Image to OpenCV format
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_update_time["camera"] = time.time()
            # Emit signal for thread-safe GUI update
            self.image_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Camera conversion error: {e}")
    
    def on_targets(self, msg: TargetPosition):
        """
        Handle target positions (String placeholder version)
        TODO: Replace with on_targets_custom when using TargetPositionArray
        
        Expected format: "name1,x,y,z,dist;name2,x,y,z,dist;..."
        """
        targets_data = {}
        self.last_update_time["targets"] = time.time()
        targets_data[msg.name] = (msg.position, 0.0)
        self.targets = targets_data
        self._update_target_displays()
        # try:
            
        #     for target_str in msg.data.split(';'):
        #         if not target_str.strip():
        #             continue
        #         parts = target_str.split(',')
        #         if len(parts) >= 5:
        #             name = parts[0]
        #             x, y, z, dist = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        #             targets_data[name] = (x, y, z, dist)
            
            
        # except Exception as e:
        #     self.get_logger().error(f"Target parsing error: {e}")
    

    def on_limit_switches(self, msg: String):
        """Handle limit switch status update"""
        self.last_update_time["limit_switches"] = time.time()
        self.limit_switches_indicator.update_status(msg.data)
    
    def on_safety_status(self, msg: String):
        """Handle safety status update"""
        self.safety_status = msg.data
        self.last_update_time["safety"] = time.time()
        self.safety_indicator.update_status(msg.data)
    
    def on_homing_status(self, msg: String):
        """Handle homing status update"""
        self.homing_status = msg.data
        self.last_update_time["homing"] = time.time()
        self.homing_indicator.update_status(msg.data)
    
    def on_path_planning_status(self, msg: String):
        """Handle path planning execution status update"""
        self.path_planning_status = msg.data
        self.last_update_time["path_planning"] = time.time()
        self.path_indicator.update_status(msg.data)
    
    # ========== Helper Methods ==========
    
    def _update_camera_display(self, cv_image):
        """Thread-safe camera display update"""
        self.camera_feed.update_image(cv_image)
    
    def _update_target_displays(self):
        """
        Update target display widgets
        Sorts targets by distance (closest to furthest)
        """
        # Clear existing target displays
        # while self.targets_layout.count() > 1:  # Keep the stretch at the end
        #     item = self.targets_layout.takeAt(0)
        #     if item.widget():
        #         item.widget().deleteLater()
        
        # # Sort targets by distance (closest first)
        # sorted_targets = sorted(self.targets.items(), key=lambda x: x[1][3])
        
        # Create new display widgets for each target
        
        for name in self.targets.keys():
            if name not in self.target_displays.keys():
                print("creating display")
                self.target_displays[name] = TargetDisplay(name)
                self.targets_layout.insertWidget(self.targets_layout.count() - 1, self.target_displays[name])
            
            self.target_displays[name].update_data(self.targets[name][0].x, self.targets[name][0].y, self.targets[name][0].z, self.targets[name][1])
            
            # Apply visibility toggles
            if not self.dist_check.isChecked():
                self.target_displays[name].dist_label.hide()
            if not self.x_check.isChecked():
                self.target_displays[name].x_label.hide()
            if not self.y_check.isChecked():
                self.target_displays[name].y_label.hide()
            if not self.z_check.isChecked():
                self.target_displays[name].z_label.hide()
            
            # Insert before the stretch
            
    
    def toggle_target_display(self):
        """Toggle visibility of target data fields based on checkboxes"""
        for i in range(self.targets_layout.count()):
            item = self.targets_layout.itemAt(i)
            if item and item.widget() and isinstance(item.widget(), TargetDisplay):
                display = item.widget()
                display.dist_label.setVisible(self.dist_check.isChecked())
                display.x_label.setVisible(self.x_check.isChecked())
                display.y_label.setVisible(self.y_check.isChecked())
                display.z_label.setVisible(self.z_check.isChecked())
    
    def _fmt_joints(self, joints: list) -> str:
        """
        Format joint angles for display
        Converts radians to degrees
        """
        if not joints:
            return "(no data)"
        parts = []
        for i, j in enumerate(joints, start=1):
            try:
                jf = float(j)
                angle_deg = math.degrees(jf)
                parts.append(f"J{i}={angle_deg:.1f}°" if math.isfinite(jf) else f"J{i}=?")
            except Exception:
                parts.append(f"J{i}=?")
        return " | ".join(parts)
    
    def _mark_stale_if_needed(self):
        """
        Check if any data streams have gone stale
        Updates displays to show "(waiting...)" for stale data
        """
        now = time.time()
        t = self.stale_after_s
        
        if now - self.last_update_time["state"] > t:
            self.state_indicator.set_stale()
        if now - self.last_update_time["curr_joints"] > t:
            self.curr_joints_indicator.set_stale()
        if now - self.last_update_time["target_joints"] > t:
            self.target_joints_indicator.set_stale()
        if now - self.last_update_time["curr_pose"] > t:
            self.pose_indicator.set_stale()
        if now - self.last_update_time["camera"] > t:
            self.camera_feed.setText("CAMERA FEED\n(waiting...)")
        if now - self.last_update_time["limit_switches"] > t:
            self.limit_switches_indicator.set_stale()
        if now - self.last_update_time["safety"] > t:
            self.safety_indicator.set_stale()
        if now - self.last_update_time["homing"] > t:
            self.homing_indicator.set_stale()
        if now - self.last_update_time["path_planning"] > t:
            self.path_indicator.set_stale()


def main():
    """Main entry point"""
    rclpy.init()
    app = QApplication(sys.argv)
    
    # Create GUI window
    win = ArmGUI(stale_after_s=1.0)
    
    # ROS spin timer - processes ROS callbacks
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(win, timeout_sec=0.01))
    ros_timer.start(10)  # 100Hz update rate
    
    win.show()
    
    try:
        code = app.exec()
    finally:
        win.destroy_node()
        rclpy.shutdown()
    
    sys.exit(code)


if __name__ == "__main__":
    main()