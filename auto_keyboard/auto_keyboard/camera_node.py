from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
#from arm_msgs.msg import KeyboardCorners   # Replaced with KeyboardCoords.msg
from geometry_msgs.msg import Point
from arm_msgs.msg import KeyboardCoords

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
        self.last_depth_frame = np.ndarray

        # TODD: import CameraInfo message.
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        self.camera_info = rs.extstruct.rs_intrinsics()
        # load a YOLO model
        # TODO: change to pull from config file
        self.keyboard_model = YOLO('src/rsx_arm/auto_keyboard/custom_yolo.pt')
        self.aruco_model = YOLO('src/rsx_arm/auto_keyboard/aruco_yolo.pt')

        self.keyboard_coords_pub = self.create_publisher(KeyboardCoords, 'keyboard_corners', 10)

        self.get_logger().info("Camera YOLO node started.")

        self.keyboard_keys_pub = self.create_publisher(Float32MultiArray, 'keyboard_keys_global', 10)

        #==============================================================
        #                KEYBOARD Key Measurememts
        #==============================================================
        # Local keyboard map (example). Fill this dictionary when you have all keys measured.
        # **** Assumption:
        #           Keyboard key measurements taken from bottom left corner of keyboard.
        # TODO: fill out the dictionary with real measurements (Fixed order according to KeyboardCoords.msg) (arm_msgs/msg/KeyboardCoords.msg)
        self.keyboard_keys_local = {
            # NOTE: MUST exactly match field names in KeyboardCoords.msg   (arm_msgs/msg/KeyboardCoords.msg)
            "A": np.array([4.5, 5.0], dtype=float),
            "B": np.array([13, 3.5], dtype=float),

            # Example for numbers AFTER you fix msg names:
            "Num1": np.array([0.0, 0.0], dtype=float),  # TODO replace
            "Num0": np.array([0.0, 0.0], dtype=float),  # TODO replace

            # Example for special keys:
            "Enter": np.array([0.0, 0.0], dtype=float), # TODO replace
            "Plus": np.array([0.0, 0.0], dtype=float),  # TODO replace
            "Cmd": np.array([0.0, 0.0], dtype=float),   # TODO replace
        }
        # Fixed order so the subscriber knows how to decode the array.
        self.key_order = list(self.keyboard_keys_local.keys())

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
                
                msg = self.get_keyboard_all_in_global_frame(aruco_corners, self.last_frame)
                # If all corners [x, y, z] are not identified, do not publish, will cause crash
                if msg is not None:
                    self.keyboard_coords_pub.publish(msg)
                else:
                    # Optional: log occasionally, not every frame
                    self.get_logger().warn("KeyboardCoords not published (missing/invalid corner depth).")

                #self._publish_keys_from_corners(msg)  # not needed, already accomplished in self.get_keyboard_all_in_global_frame()
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
    def get_keyboard_all_in_global_frame(self, boxes, frame):
        """
        Gets all corners + keys of keyboard in global/target frame.
        Returns:
            KeyboardCoords if all 4 corners are valid, otherwise None.
        """
        # Guard: if depth doesn't have value yet, can't do computations, return None msg
        if self.last_depth_frame is None:
            return None

        #corners = KeyboardCoords()
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

        # Safety: Check if all depth has been identified, if any value is still NaN, cannot calculate key coords, return None to be safe
        if any(c is None for c in corners):
            return None

        msg = KeyboardCoords()
        msg.tl = Point(x=float(corners[0][0]), y=float(corners[0][1]), z=float(corners[0][2]))
        msg.tr = Point(x=float(corners[1][0]), y=float(corners[1][1]), z=float(corners[1][2]))
        msg.br = Point(x=float(corners[2][0]), y=float(corners[2][1]), z=float(corners[2][2]))
        msg.bl = Point(x=float(corners[3][0]), y=float(corners[3][1]), z=float(corners[3][2]))

        # Uncomment if your global frame is not camera's global frame but arm's base link or something else
        # if self.use_tf:
        #     msg = self._transform_corners_to_target_frame(msg)

        msg = self._populate_keys_in_msg(msg)  # Fill out all other keys in the message
        return msg
        
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

    def _unit(self, v: np.ndarray) -> np.ndarray:
        """ "Normalize a vector to a unit vector"""
        n = np.linalg.norm(v)
        if n == 0:
            raise ValueError("Zero-length vector; keyboard corners are degenerate.")
        return v / n

    def find_key_pt_in_global_frame_from_corners(self,
        key_xy_local: np.ndarray,
        tl: np.ndarray,
        tr: np.ndarray,
        br: np.ndarray,
        bl: np.ndarray,
    ) -> np.ndarray:
        """
        Compute a key point in the global/target frame using known keyboard corners.
        Returns:
            key_pt_global (np.ndarray): Global coordinates of the key point.
        Assumptions:
        - Keyboard local frame origin is at bottom-left corner (bl)
        - +x_local goes from bl -> br along the bottom edge
        - +y_local goes from bl -> tl along the left edge
        - key_xy_local is [x_local, y_local] in same units as corner distances
        """
        x_local, y_local = float(key_xy_local[0]), float(key_xy_local[1])

        u_bottom = self._unit(br - bl)  # unit vector along bottom edge
        u_left   = self._unit(tl - bl)  # unit vector along left edge

        key_pt_global = bl + x_local * u_bottom + y_local * u_left  # Simple vector addition to go from pt. bl to key point in global frame
        return key_pt_global

    #TODO: build KeyboardCoords message for ROS2

    def _populate_keys_in_msg(self, msg: KeyboardCoords) -> KeyboardCoords:
        # corners must already be in target frame if you want keys in target frame
        tl = np.array([msg.tl.x, msg.tl.y, msg.tl.z], dtype=float)
        tr = np.array([msg.tr.x, msg.tr.y, msg.tr.z], dtype=float)
        br = np.array([msg.br.x, msg.br.y, msg.br.z], dtype=float)
        bl = np.array([msg.bl.x, msg.bl.y, msg.bl.z], dtype=float)

        for field_name, key_local in self.keyboard_keys_local.items():
            key_global = self.find_key_pt_in_global_frame_from_corners(key_local, tl, tr, br, bl)
            setattr(msg, field_name, Point(
                x=float(key_global[0]),
                y=float(key_global[1]),
                z=float(key_global[2]),
            ))
        return msg


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
