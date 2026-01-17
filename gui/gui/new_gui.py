#!/usr/bin/env python3
import sys
import os
import cv2
import numpy as np
from threading import Thread

# PyQt5
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriberNode(Node):
    def __init__(self, signal):
        super().__init('cv_subscriber_node')
        self.bridge = CvBridge()
        self.signal = signal
        self.subscription = self.create_subscription(Image, 'camera/annotated_frame', self.image_callback, 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Send to PyQt thread via signal
        self.signal.emit(cv_img)

class SimpleWindow(QWidget):
    # to receive cv images from CameraNode
    cv_signal = pyqtSignal(np.ndarray)

    def __init__(self, signal):
        super().__init__()
        self.setWindowTitle("Simple PyQt Window")
        self.setGeometry(200, 200, 800, 500)  # x, y, width, height

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # ================
        # Camera Feed
        # ================

        self.cv_label = QLabel("Waiting for camera feed...")
        self.cv_label.setAlignment(Qt.AlignCenter)
        
        # to mae video boundaries visible:
        self.cv_label.setStyleSheet("background-color: black; color: white")
        # add label to vertical stack
        self.layout.addWidget(self.cv_label)
        
        self.cv_signal.connect(self.update_image)

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Convert OpenCV BGR image to QPixmap and update cv_label."""
        try:
            # get camera feed image dimensions
            h, w, ch = cv_img.shape #ch is number of color channels
            bytes_per_line = ch*w

            # since cv_bridge data already BGR...
            qt_image = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)

            # convert to pixmap and scale for displaying in the GUI window
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.cv_label.width(),
                self.cv_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.cv_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"GUI Update Error: {e}")



def main():
    rclpy.init()
    
    app = QApplication(sys.argv)
    window = SimpleWindow()

    camera_node = CameraSubscriberNode(window.cv_signal)
    # sprin ROS2 in a background thread
    ros_thread = Thread(target=rclpy.spin, args=(camera_node,), daemon=True)
    ros_thread.start()

    window.show()

    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()