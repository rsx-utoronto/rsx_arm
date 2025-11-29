from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import ultralytics
from ultralytics import YOLO
# subscribe to camera data node (sensor_msg/msg/Image.msg)
# cv_bridge image data converted to numpy array

class CameraNode(Node):

    def __init__(self):
        # create camera node
        super().__init__('camera')
        self.camera_pub = self.create_publisher(Float32MultiArray, 'camera_numpy_array_topic', 10)

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.callback, 10)
        self.last_frame = np.ndarray

        # load a YOLO model
        self.model = YOLO('yolov8n.pt')

        self.get_logger().info("Camera YOLO node started.")

    def callback(self, data):
        try:
            # convert Image to numpy array
            # "mono8": 8-bit grayscale image.
            # "bgr8": 8-bit color image with Blue-Green-Red channel order (common in OpenCV).
            # "rgb8": 8-bit color image with Red-Green-Blue channel order. 
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.last_frame = cv_image

            # Run YOLO inference
            results  = self.model(cv_image)

            # get bounding boxes
            annotated_frame = results[0].plot()
            # print("heyo")
            # Display live stream
            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)

        except CvBridgeError as e: 
            # HI DUDE
            rclpy.logerr(e)
            return

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
