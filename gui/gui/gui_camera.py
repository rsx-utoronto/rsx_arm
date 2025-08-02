import rclpy
from rclpy.node import Node
import cv2
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QImage
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np

class ROSVideoSubscriber(QObject):
    frame_received = pyqtSignal(QImage)

    def __init__(self, node, topic_name, compressed=False):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.compressed = compressed
        msg_type = CompressedImage if compressed else Image
        self.sub = self.node.create_subscription(msg_type, topic_name, self.callback, 10)

    def callback(self, msg):
        try:
            if self.compressed:
                # Handle compressed images
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # Handle uncompressed images
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Convert the CV2 image to RGB format for PyQt
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Convert the image to a format suitable for PyQt (QImage)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Emit the converted QImage to update the GUI
            self.frame_received.emit(qt_image)

        except Exception as e:
            self.node.get_logger().error(f"Error processing image: {e}")