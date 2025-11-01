from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2

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

        self.model = YOLO("yolo11n.pt")
    def callback(self, data):
        try:
            # convert Image to numpy array
            # "mono8": 8-bit grayscale image.
            # "bgr8": 8-bit color image with Blue-Green-Red channel order (common in OpenCV).
            # "rgb8": 8-bit color image with Red-Green-Blue channel order. 
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.last_frame = cv_image

            # get YOLO11 model results
            results = self.model(cv_image)

            for result in results:
                boxes = result.boxes #bounding boxes
                scores = result.boxes.conf #confidence in boxes
                classes = result.boxes.cls #determine class IDs

                for bounding_box, score, cls in zip(boxes, scores, classes):
                    self.get_logger().info(f"Detected class {int(cls)} with a confidence score of: {float(score)}")

                annotated_frame = results[0].plot()
                cv2.imshow("YOLO Ultralytics Detection", annotated_frame)
                cv2.waitKey(1)

        except CvBridgeError as e:
            rclpy.logerr(e)
            return

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.destroy_node()

if __name__ == '__main__':
    main()
