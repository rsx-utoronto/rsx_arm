from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
import cv2

# subscribe to camera data node (sensor_msg/msg/Image.msg)
# publish cv_bridge image data converted to numpy array

class CameraNode(Node):

    def __init__(self):
        # create camera node
        super().__init__('camera')
        self.camera_pub = self.create_publisher(Float32MultiArray, 'camera_numpy_array_topic', 10)

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.callback, 10)
        self.last_frame = np.ndarray
    def callback(self, data):
        # try:
            # convert Image to numpy array
            # "mono8": 8-bit grayscale image.
            # "bgr8": 8-bit color image with Blue-Green-Red channel order (common in OpenCV).
            # "rgb8": 8-bit color image with Red-Green-Blue channel order. 
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.last_frame = cv_image
        print("frame received!")
        # self.camera_pub.publish(cv_image)

        # except CvBridgeError as e:
            # rclpy.logerr(e)
            # return

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.destroy_node()

if __name__ == '__main__':
    main()
