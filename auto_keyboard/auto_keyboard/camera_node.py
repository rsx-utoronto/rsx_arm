from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from arm_msgs.msg import KeyboardCorners
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import ultralytics
from ultralytics import YOLO
import pyrealsense2 as rs
# subscribe to camera data node (sensor_msg/msg/Image.msg)
# cv_bridge image data converted to numpy array

class CameraNode(Node):

    def __init__(self):
        # create camera node
        super().__init__('camera')
        self.camera_pub = self.create_publisher(Float32MultiArray, 'camera_numpy_array_topic', 10)

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.update_depth_map, 10)
        self.last_frame = np.ndarray

        self.model = YOLO("yolo11n.pt")

        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100) # add params
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
                # depth = depth_image[center_y, center_x]  # in meters or mm
                depth = 1.0  # Placeholder depth value (testing with static image)

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

            # Run YOLO inference
            # TODO: might just not use the keybaord detection i don't have time to do this lol
            keyboard_results  = self.keyboard_model(cv_image)
            aruco_results = self.aruco_model(cv_image)
            if len(aruco_results.boxes) == 4:
                self.get_logger().info("All ArUco markers detected, approximating keyboard corners")
                aruco_corners = []
                for box in aruco_results.boxes:
                    aruco_corners.append(box.xyxy)
                
                msg = self.get_keyboard_corners(aruco_corners, self.last_frame)
                self.keyboard_corner_pub.publish(msg)
                return

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
    def camera_info_callback(self, data):
        # parse camera info data into rs_intrinsics
        self.camera_info.width = data.width
        self.camera_info.height = data.height
        self.camera_info.ppx = data.k[2]
        self.camera_info.ppy = data.k[5]

        self.camera_info.fx = data.k[0]
        self.camera_info.fy = data.k[4]
        self.camera_info.model = rs.extstruct.rs_distortion.none
        self.camera_info.coeffs = [0, 0, 0, 0, 0]
    def update_depth_map(self, data):
        self.last_depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    def get_keyboard_corners(self, boxes, frame):
        corners = KeyboardCorners()
        boxes_x = []
        boxes_y = []
        for box in boxes:
            x1, y1, x2, y2 = box[0]
            boxes_x.append((x1 + x2) / 2)
            boxes_y.append((y1 + y2) / 2)
        # Top-left
        tl_index = np.argmin(np.array(boxes_x) + np.array(boxes_y))
        # Top-right
        tr_index = np.argmin(np.array(boxes_y) - np.array(boxes_x))
        # Bottom-right
        br_index = np.argmax(np.array(boxes_x) + np.array(boxes_y))
        # Bottom-left
        bl_index = np.argmax(np.array(boxes_y) - np.array(boxes_x))
        keyboard_tl = boxes[tl_index][1]
        keyboard_tr = (boxes[tr_index][0][0], boxes[tr_index][1][1])
        keyboard_br = boxes[br_index][0]
        keyboard_bl = (boxes[bl_index][0][0], boxes[bl_index][1][1])

        corners = self.get_positions([keyboard_tl, keyboard_tr, keyboard_br, keyboard_bl], frame)
        return corners
    def get_positions(self, corners, frame):
        positions = []
        for corner in corners:
            point = self.deproject_pixel_to_point(corner, self.last_depth_frame)
            positions.append(point)
        return positions

    def deproject_pixel_to_point(self, pixel, depth_frame):
        depth = depth_frame[int(pixel[1]), int(pixel[0])]
        if depth == 0:
            return None

        point = rs.rs2_deproject_pixel_to_point(self.camera_info, [pixel[0], pixel[1]], depth)
        return point
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
