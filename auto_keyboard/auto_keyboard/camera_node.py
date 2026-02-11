#!/usr/bin/env python3
"""
Camera Node for ArUco-based Keyboard Detection
Uses OpenCV's native ArUco detection instead of YOLO for better accuracy
Calculates 3D positions using RealSense depth data and camera intrinsics
"""

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from arm_msgs.msg import KeyboardCorners
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import pyrealsense2 as rs


class CameraNode(Node):
    """
    Camera node for detecting ArUco markers and computing keyboard corner positions
    """

    def __init__(self):
        super().__init__('camera_aruco_node')
        
        # Publishers
        self.camera_pub = self.create_publisher(
            Float32MultiArray, 'camera_numpy_array_topic', 10
        )
        self.keyboard_corner_pub = self.create_publisher(
            KeyboardCorners, 'keyboard_corners', 10
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_rect_raw', 
            self.image_callback, 
            10
        )
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            self.update_depth_map, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/aligned_depth_to_color/camera_info', 
            self.camera_info_callback, 
            10
        )
        
        # Frame storage
        self.last_frame = None
        self.last_depth_frame = None
        
        # Camera intrinsics (RealSense format)
        self.camera_info = None
        self.intrinsics_ready = False
        
        # Camera intrinsics (OpenCV format) for manual deprojection
        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None
        
        # ArUco detection setup
        self.setup_aruco_detector()
        
        # ArUco marker IDs for keyboard corners
        # NOTE: We'll identify corners by position, not by specific IDs
        # Any 4 markers will work - we determine which is which by spatial location
        self.require_specific_ids = False  # Set to True if you want to enforce specific IDs
        self.expected_marker_ids = {0, 1, 2, 3}  # Only used if require_specific_ids=True
        
        # Visualization flag
        self.visualize = True
        
        # Depth scale (typically 0.001 for RealSense = mm to meters)
        self.depth_scale = 0.001
        
        self.get_logger().info("Camera ArUco node started.")
        self.get_logger().info(f"Expected ArUco marker IDs: {self.marker_ids}")

    def setup_aruco_detector(self):
        """Setup OpenCV ArUco detector with appropriate dictionary"""
        # Choose ArUco dictionary - adjust based on your markers
        # Common options:
        # - cv2.aruco.DICT_4X4_50: 4x4 bits, 50 markers
        # - cv2.aruco.DICT_5X5_100: 5x5 bits, 100 markers
        # - cv2.aruco.DICT_6X6_250: 6x6 bits, 250 markers
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Create detector parameters
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Tune parameters for better detection (optional)
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        
        # Create detector
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, 
            self.aruco_params
        )
        
        self.get_logger().info("ArUco detector initialized with DICT_4X4_50")

    def camera_info_callback(self, data):
        """
        Parse camera intrinsics from ROS CameraInfo message
        K matrix layout: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        """
        if not self.intrinsics_ready:
            # Store as RealSense intrinsics structure
            self.camera_info = rs.intrinsics()
            self.camera_info.width = data.width
            self.camera_info.height = data.height
            self.camera_info.ppx = data.k[2]  # cx
            self.camera_info.ppy = data.k[5]  # cy
            self.camera_info.fx = data.k[0]
            self.camera_info.fy = data.k[4]
            self.camera_info.model = rs.distortion.none
            self.camera_info.coeffs = [0, 0, 0, 0, 0]
            
            # Also store for manual deprojection
            self.fx = data.k[0]
            self.fy = data.k[4]
            self.ppx = data.k[2]
            self.ppy = data.k[5]
            
            self.intrinsics_ready = True
            
            self.get_logger().info(
                f"Camera intrinsics loaded: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.ppx:.2f}, cy={self.ppy:.2f}"
            )

    def update_depth_map(self, data):
        """Update the depth frame from ROS message"""
        try:
            # Depth is typically uint16 in millimeters
            self.last_depth_frame = self.bridge.imgmsg_to_cv2(
                data, desired_encoding="passthrough"
            )
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def image_callback(self, data):
        """Main callback for processing RGB images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.last_frame = cv_image
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(cv_image)
            
            if ids is not None and len(ids) >= 4:
                self.get_logger().info(
                    f"Detected {len(ids)} ArUco markers: {ids.flatten().tolist()}", 
                    throttle_duration_sec=2.0
                )
                
                # Process markers to get keyboard corners
                keyboard_corners_msg = self.process_aruco_markers(
                    corners, ids, cv_image
                )
                
                if keyboard_corners_msg is not None:
                    self.keyboard_corner_pub.publish(keyboard_corners_msg)
                    self.get_logger().info(
                        "Published keyboard corners", 
                        throttle_duration_sec=5.0
                    )
            
            # Visualization
            if self.visualize and ids is not None:
                display_frame = cv_image.copy()
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                cv2.imshow("ArUco Detection", display_frame)
                cv2.waitKey(1)
            elif self.visualize:
                cv2.imshow("ArUco Detection", cv_image)
                cv2.waitKey(1)
                
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def process_aruco_markers(self, corners, ids, image):
        """
        Process detected ArUco markers to compute keyboard corner positions
        Identifies corners by spatial position (top-left, top-right, etc.)
        
        Args:
            corners: List of detected marker corners
            ids: Array of marker IDs
            image: Current RGB frame
            
        Returns:
            KeyboardCorners message with 3D positions, or None if failed
        """
        if not self.intrinsics_ready:
            self.get_logger().warn(
                "Camera intrinsics not ready", 
                throttle_duration_sec=5.0
            )
            return None
        
        if self.last_depth_frame is None:
            self.get_logger().warn(
                "Depth frame not available", 
                throttle_duration_sec=5.0
            )
            return None
        
        # Need exactly 4 markers for keyboard corners
        if len(ids) != 4:
            self.get_logger().warn(
                f"Need exactly 4 markers, found {len(ids)}", 
                throttle_duration_sec=2.0
            )
            return None
        
        # Optional: Check if specific IDs are required
        if self.require_specific_ids:
            detected_ids = set(ids.flatten())
            if not self.expected_marker_ids.issubset(detected_ids):
                missing = self.expected_marker_ids - detected_ids
                self.get_logger().warn(
                    f"Missing expected marker IDs: {missing}", 
                    throttle_duration_sec=2.0
                )
                return None
        
        # Calculate center position of each marker
        marker_centers = []
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            marker_corners_2d = corners[i][0]  # Shape: (4, 2)
            
            # Calculate center of the marker
            center = np.mean(marker_corners_2d, axis=0)
            
            marker_centers.append({
                'id': marker_id,
                'center': center,
                'x': center[0],
                'y': center[1]
            })
        
        # Identify corners based on spatial position
        # Camera coordinates: origin at top-left, x increases right, y increases down
        
        # Top-left: smallest x+y (closest to origin)
        tl_marker = min(marker_centers, key=lambda m: m['x'] + m['y'])
        
        # Top-right: smallest y-x (top of image, but rightmost)
        tr_marker = min(marker_centers, key=lambda m: m['y'] - m['x'])
        
        # Bottom-right: largest x+y (farthest from origin)
        br_marker = max(marker_centers, key=lambda m: m['x'] + m['y'])
        
        # Bottom-left: largest y-x (bottom of image, but leftmost)
        bl_marker = max(marker_centers, key=lambda m: m['y'] - m['x'])
        
        # Log the identification
        self.get_logger().info(
            f"Identified corners - TL: ID {tl_marker['id']}, "
            f"TR: ID {tr_marker['id']}, "
            f"BR: ID {br_marker['id']}, "
            f"BL: ID {bl_marker['id']}", 
            throttle_duration_sec=10.0
        )
        
        # Extract 2D positions
        keyboard_corners_2d = [
            tl_marker['center'],
            tr_marker['center'],
            br_marker['center'],
            bl_marker['center']
        ]
        
        # Convert 2D image coordinates to 3D positions
        keyboard_corners_3d = self.get_3d_positions(keyboard_corners_2d)
        
        if keyboard_corners_3d is None:
            return None
        
        # Validate the corner ordering makes sense
        if not self.validate_corner_geometry(keyboard_corners_3d):
            self.get_logger().warn(
                "Corner geometry validation failed - check marker placement",
                throttle_duration_sec=5.0
            )
            return None
        
        # Create and populate message
        msg = KeyboardCorners()
        
        # Assuming KeyboardCorners has fields like:
        # top_left, top_right, bottom_right, bottom_left
        # Each being a Point or similar with x, y, z
        
        try:
            msg.top_left.x = float(keyboard_corners_3d[0][0])
            msg.top_left.y = float(keyboard_corners_3d[0][1])
            msg.top_left.z = float(keyboard_corners_3d[0][2])
            
            msg.top_right.x = float(keyboard_corners_3d[1][0])
            msg.top_right.y = float(keyboard_corners_3d[1][1])
            msg.top_right.z = float(keyboard_corners_3d[1][2])
            
            msg.bottom_right.x = float(keyboard_corners_3d[2][0])
            msg.bottom_right.y = float(keyboard_corners_3d[2][1])
            msg.bottom_right.z = float(keyboard_corners_3d[2][2])
            
            msg.bottom_left.x = float(keyboard_corners_3d[3][0])
            msg.bottom_left.y = float(keyboard_corners_3d[3][1])
            msg.bottom_left.z = float(keyboard_corners_3d[3][2])
            
            return msg
        except Exception as e:
            self.get_logger().error(
                f"Error creating KeyboardCorners message: {e}. "
                f"Check message definition compatibility."
            )
            return None

    def validate_corner_geometry(self, corners_3d):
        """
        Validate that the detected corners form a reasonable quadrilateral
        
        Args:
            corners_3d: List of 4 corners [TL, TR, BR, BL] as [x, y, z]
            
        Returns:
            True if geometry is valid, False otherwise
        """
        if len(corners_3d) != 4:
            return False
        
        tl, tr, br, bl = [np.array(c) for c in corners_3d]
        
        # Check 1: All corners should have similar Z depth (planar object)
        z_values = [c[2] for c in corners_3d]
        z_std = np.std(z_values)
        z_mean = np.mean(z_values)
        
        if z_std > 0.1:  # More than 10cm variation in depth
            self.get_logger().warn(
                f"Corners not planar: Z std={z_std:.3f}m (expected <0.1m)",
                throttle_duration_sec=5.0
            )
            return False
        
        # Check 2: Width should be reasonable (typical keyboard is 0.3-0.5m wide)
        width_top = np.linalg.norm(tr - tl)
        width_bottom = np.linalg.norm(br - bl)
        
        if width_top < 0.1 or width_top > 1.0:
            self.get_logger().warn(
                f"Top width unrealistic: {width_top:.3f}m (expected 0.1-1.0m)",
                throttle_duration_sec=5.0
            )
            return False
        
        # Check 3: Height should be reasonable (typical keyboard is 0.1-0.2m tall)
        height_left = np.linalg.norm(bl - tl)
        height_right = np.linalg.norm(br - tr)
        
        if height_left < 0.05 or height_left > 0.5:
            self.get_logger().warn(
                f"Left height unrealistic: {height_left:.3f}m (expected 0.05-0.5m)",
                throttle_duration_sec=5.0
            )
            return False
        
        # Check 4: Aspect ratio should be reasonable
        aspect_ratio = width_top / height_left
        if aspect_ratio < 1.5 or aspect_ratio > 10.0:
            self.get_logger().warn(
                f"Aspect ratio unrealistic: {aspect_ratio:.2f} (expected 1.5-10.0)",
                throttle_duration_sec=5.0
            )
            # Don't fail on this - just warn
        
        return True
    
    def get_3d_positions(self, pixel_coords):
        """
        Convert list of 2D pixel coordinates to 3D positions
        
        Args:
            pixel_coords: List of (x, y) pixel coordinates
            
        Returns:
            List of (x, y, z) 3D positions in meters, or None if any failed
        """
        positions_3d = []
        
        for pixel in pixel_coords:
            pos_3d = self.deproject_pixel_to_point(pixel)
            
            if pos_3d is None:
                self.get_logger().warn(
                    f"Failed to deproject pixel {pixel}", 
                    throttle_duration_sec=2.0
                )
                return None
            
            positions_3d.append(pos_3d)
        
        return positions_3d

    def deproject_pixel_to_point(self, pixel):
        """
        Deproject 2D pixel coordinate to 3D point using depth and camera intrinsics
        
        Args:
            pixel: (x, y) pixel coordinate
            
        Returns:
            (x, y, z) 3D point in meters, or None if depth invalid
        """
        # Ensure pixel coordinates are within bounds
        x, y = int(pixel[0]), int(pixel[1])
        
        if (x < 0 or x >= self.last_depth_frame.shape[1] or 
            y < 0 or y >= self.last_depth_frame.shape[0]):
            self.get_logger().warn(f"Pixel {pixel} out of bounds")
            return None
        
        # Get depth value at pixel (typically in millimeters for RealSense)
        depth_raw = self.last_depth_frame[y, x]
        
        if depth_raw == 0:
            self.get_logger().warn(
                f"Zero depth at pixel ({x}, {y})", 
                throttle_duration_sec=2.0
            )
            return None
        
        # Convert depth to meters
        depth_m = float(depth_raw) * self.depth_scale
        
        # Method 1: Use RealSense deprojection function (recommended)
        try:
            point = rs.rs2_deproject_pixel_to_point(
                self.camera_info, 
                [float(x), float(y)], 
                depth_m
            )
            return point
        except Exception as e:
            self.get_logger().warn(
                f"RealSense deprojection failed: {e}, using manual method"
            )
        
        # Method 2: Manual deprojection (fallback)
        # Using pinhole camera model:
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        
        X = (x - self.ppx) * depth_m / self.fx
        Y = (y - self.ppy) * depth_m / self.fy
        Z = depth_m
        
        return [X, Y, Z]

    def get_average_depth(self, pixel, window_size=5):
        """
        Get average depth in a window around the pixel to reduce noise
        
        Args:
            pixel: (x, y) pixel coordinate
            window_size: Size of averaging window
            
        Returns:
            Average depth value in raw units, or 0 if invalid
        """
        x, y = int(pixel[0]), int(pixel[1])
        half_window = window_size // 2
        
        # Define window bounds
        y_min = max(0, y - half_window)
        y_max = min(self.last_depth_frame.shape[0], y + half_window + 1)
        x_min = max(0, x - half_window)
        x_max = min(self.last_depth_frame.shape[1], x + half_window + 1)
        
        # Extract window
        depth_window = self.last_depth_frame[y_min:y_max, x_min:x_max]
        
        # Filter out zero values
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) == 0:
            return 0
        
        # Return median for robustness against outliers
        return np.median(valid_depths)


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