import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.pub = self.create_publisher(Image, '/camera/camera/color/image_rect_raw', 10)
        self.bridge = CvBridge()

        # Full absolute path to your image
        img_path = '/absolute/path/to/aruco.jpg'
        
        # Load image using OpenCV
        img = cv2.imread(img_path)
        
        # Check if image loaded successfully
        if img is None:
            self.get_logger().error(f"Failed to load image at {img_path}")
            # Create a dummy array to avoid cv_bridge crashing
            img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Ensure image is a proper NumPy array of type uint8 (required by cv_bridge)
        if not isinstance(img, np.ndarray):
            img = np.array(img, dtype=np.uint8)
        elif img.dtype != np.uint8:
            img = img.astype(np.uint8)

        self.msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

        # Timer to publish image every second
        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        self.pub.publish(self.msg)
        self.get_logger().info("Publishing test image")

def main():
    rclpy.init()
    node = TestImagePublisher()
    rclpy.spin(node)
