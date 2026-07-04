#!/usr/bin/env python3
import sys
import os
import cv2
import numpy as np
from threading import Thread

# PyQt6
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QWidget, QPushButton
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
import PyQt6.QtWidgets as QtWidgets

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MainWindow(QMainWindow):
    # to receive cv images from CameraNode
    cv_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        QMainWindow.__init__(self)
        self.node = rclpy.create_node("MainWindow")

        self.setWindowTitle("RSX ARM GUI")
        self.setGeometry(100, 100, 800, 600)  # x, y, width, height

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)

        self.layout = QVBoxLayout(self.centralWidget)

        # ================
        # CAMERA FEED
        # ================

        self.bridge = CvBridge()
        self.subscription_camera = None # so we can toggle camera subscription if we want ¯\_(ツ)_/¯

        # CAMERA FEED HEADER
        self.header_label = QLabel("Camera Feed")
        self.header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # make it bold and slightly larger
        self.header_label.setStyleSheet("font-size: 16px; font-weight: bold; margin-bottom: 5px;")
        self.layout.addWidget(self.header_label)
        
        # START/STOP CAMERA FEED BUTTON
        self.toggle_btn = QPushButton("Stop Camera")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setChecked(True)
        self.toggle_btn.clicked.connect(self.toggle_feed)
        self.toggle_btn.setFixedWidth(200)
        self.toggle_btn.setStyleSheet("""
            QPushButton {
                background-color: #ffcccc; 
                border: 2px solid red;
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #ff9999;
            }
        """)
        self.layout.addWidget(self.toggle_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        # CAMERA FEED
        self.cv_label = QLabel("Waiting for camera feed...")
        self.cv_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cv_label.setStyleSheet("background-color: black; color: white")
        self.cv_label.setMinimumSize(320,240)
        self.layout.addWidget(self.cv_label)
        # connet signal to slot
        self.cv_signal.connect(self.update_image)
        # start camera immediately
        self.start_camera_subscription()

    # CAMERA
    def start_camera_subscription(self):
        if self.subscription_camera is None:
            self.subscription_camera = self.node.create_subscription(Image, 'camera/annotated_frame', self.image_callback, 10)
    def stop_camera_subscription(self):
        if self.subscription_camera is not None:
            self.node.destroy_subscription(self.subscription_camera)
            self.subscription_camera = None
    def toggle_feed(self):
        if self.toggle_btn.isChecked():
            self.toggle_btn.setText("Stop Camera")
            self.toggle_btn.setStyleSheet("background-color: #ffcccc; border: 2px solid red; border-radius: 5px; padding: 5px;")
            self.cv_label.setText("Waiting for camera feed...")
            self.start_camera_subscription()
        else:
            self.toggle_btn.setText("Start Camera")
            self.toggle_btn.setStyleSheet("background-color: #ccffcc; border: 2px solid green; border-radius: 5px; padding: 5px;")
            self.cv_label.clear() # clear last frame from image feed
            self.cv_label.setText("Feed Paused")
            self.stop_camera_subscription()
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Send to PyQt thread via signal
            self.cv_signal.emit(cv_img)
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge Error: {e}")

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Convert OpenCV BGR image to QPixmap and update cv_label."""
        if not self.toggle_btn.isChecked():
            return
        try:
            # get camera feed image dimensions
            h, w, ch = cv_img.shape #ch is number of color channels
            bytes_per_line = ch*w

            # since cv_bridge data already BGR...
            qt_image = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)

            # convert to pixmap and scale for displaying in the GUI window
            pixmap = QPixmap.fromImage(qt_image)

            target_w = self.cv_label.width() if self.cv_label.width() > 0 else 640
            target_h = self.cv_label.height() if self.cv_label.height() > 0 else 480 # is it a 4:3 aspect ratio?
            scaled_pixmap = pixmap.scaled(target_w, target_h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)

            self.cv_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"GUI Update Error: {e}")



def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    ui = MainWindow()
    app.processEvents()
    ui.show()

    exec = MultiThreadedExecutor()
    exec.add_node(ui.node)
    
    while rclpy.ok():
        try:
            exec.wait_for_ready_callbacks(0)
            exec.spin_once()
        except:
            pass
        app.processEvents()
    
    app.quit()
    exec.remove_node(ui.node)

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()