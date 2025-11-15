from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
from cv2 import aruco
from .keyboard_utils import KeyboardCalibration

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

        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary() # add params
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Initialize keyboard calibration
        self.keyboard_calibration = KeyboardCalibration()

        # Load camera intrinsics for depth calculation (change placeholders)
        self.camera_fx = 0.0
        self.camera_fy = 0.0
        self.camera_cx = 0.0
        self.camera_cy = 0.0

    def detect_aruco_markers(self, cv_image, depth_image):
        """
        Detect Aruco markers and compute their 3D positions.

        Args:
            cv_image: BGR image
            depth_image
        """
        # Detect ArUco markers
        corners, ids, rejected = self.aruco_detector.detectMarkers(cv_image)

        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                # Get marker corners (4 corners per marker)
                marker_corners = corners[i][0]

                # Compute marker center in image coordinates
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                # TODO: Get depth value at marker center
                depth = depth_image[center_y, center_x]  # in meters or mm

                # Convert 2D image coordinates + depth to 3D camera frame
                position_3d = self.compute_3d_position(center_x, center_y, depth)

                # Update keyboard calibration with corner detection
                self.keyboard_calibration.update_corner_detection(marker_id, position_3d)

                self.get_logger().info(
                    f"Aruco marker {marker_id} detected at 3D position: {position_3d}"
                )

        return cv_image

    def compute_3d_position(self, pixel_x, pixel_y, depth):
        """
        Convert 2D pixel coordinates + depth to 3D camera frame coordinates.

        Args:
            pixel_x: X coordinate in image (pixels)
            pixel_y: Y coordinate in image (pixels)
            depth: Depth value (meters)

        Returns:
            np.array: (x, y, z) position in camera frame
        """
        # Convert from pixel coordinates to camera frame, change placeholders
        x = 0
        y = 0
        z = depth

        return np.array([x, y, z])

    def callback(self, data):
        try:
            # convert Image to numpy array
            # "mono8": 8-bit grayscale image.
            # "bgr8": 8-bit color image with Blue-Green-Red channel order (common in OpenCV).
            # "rgb8": 8-bit color image with Red-Green-Blue channel order.
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.last_frame = cv_image

            # TODO: Subscribe to depth image topic and get depth data
            depth_image = None  # Placeholder

            # Detect ArUco markers and update calibration
            cv_image = self.detect_aruco_markers(cv_image, depth_image)

            # Display calibration status
            if self.keyboard_calibration.is_calibrated:
                cv2.putText(cv_image, "Keyboard Calibrated", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Example: Get position of a specific key
                key_pos = self.keyboard_calibration.get_key_position('A')
                if key_pos is not None:
                    self.get_logger().info(f"Key 'A' position: {key_pos}", throttle_duration_sec=2.0)
            else:
                cv2.putText(cv_image, "Detecting corners...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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
