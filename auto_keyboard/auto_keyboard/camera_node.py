from sensor_msgs.msg import Image, CameraInfo
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
        self.annotated_pub = self.create_publisher(Image, 'camera/annotated_frame', 10)
        self.camera_pub = self.create_publisher(Float32MultiArray, 'camera_numpy_array_topic', 10)

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.update_depth_map, 10)
        self.last_frame = np.ndarray
        self.last_depth_frame = np.ndarray

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        # self.camera_info = rs.extstruct.rs_intrinsics()
        self.camera_info = rs.intrinsics()
        # load a YOLO model
        # TODO: change to pull from config file
        self.keyboard_model = YOLO('src/rsx_arm/auto_keyboard/custom_yolo.pt')
        # uncomment when we have the aruco YOLO modele
        # self.aruco_model = YOLO('src/rsx_arm/auto_keyboard/aruco_yolo.pt')

        self.keyboard_corner_pub = self.create_publisher(KeyboardCorners, 'keyboard_corners', 10)

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
            # TODO: might just not use the keybaord detection i don't have time to do this lol
            keyboard_results  = self.keyboard_model(cv_image)
            # aruco_results = self.aruco_model(cv_image)
            # if len(aruco_results.boxes) == 4:
            #     self.get_logger().info("All ArUco markers detected, approximating keyboard corners")
            #     aruco_corners = []
            #     for box in aruco_results.boxes:
            #         aruco_corners.append(box.xyxy)
                
            #     msg = self.get_keyboard_corners(aruco_corners, self.last_frame)
            #     self.keyboard_corner_pub.publish(msg)
            #     return

            # get bounding boxes
            annotated_frame = keyboard_results[0].plot()
            # annotated_frame = aruco_results[0].plot()
            # print("heyo")

            # Display live stream
            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)
            # which can be replaced by:
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.annotated_pub.publish(img_msg)


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
