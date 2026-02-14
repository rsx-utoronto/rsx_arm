#!/usr/bin/env python3
"""
ArUco Tag Position Tester
Detects any ArUco tags in view and overlays their 3D positions on the live stream.
"""

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import pyrealsense2 as rs


class ArucoTester(Node):

    def __init__(self):
        super().__init__('aruco_tester')

        self.bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )

        self.last_depth_frame = None
        self.intrinsics_ready = False
        self.rs_intrinsics = None
        self.fx = self.fy = self.ppx = self.ppy = None
        self.depth_scale = 0.001  # RealSense: mm -> metres

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info('ArUco tester ready.')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def camera_info_callback(self, msg):
        if self.intrinsics_ready:
            return

        self.fx   = msg.k[0]
        self.fy   = msg.k[4]
        self.ppx  = msg.k[2]
        self.ppy  = msg.k[5]

        self.rs_intrinsics = rs.intrinsics()
        self.rs_intrinsics.width  = msg.width
        self.rs_intrinsics.height = msg.height
        self.rs_intrinsics.fx     = self.fx
        self.rs_intrinsics.fy     = self.fy
        self.rs_intrinsics.ppx    = self.ppx
        self.rs_intrinsics.ppy    = self.ppy
        self.rs_intrinsics.model  = rs.distortion.none
        self.rs_intrinsics.coeffs = [0, 0, 0, 0, 0]

        self.intrinsics_ready = True
        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.ppx:.1f} cy={self.ppy:.1f}'
        )

    def depth_callback(self, msg):
        try:
            self.last_depth_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except CvBridgeError as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                center = np.mean(corners[i][0], axis=0).astype(int)
                pos3d  = self.deproject(center)
                self.draw_overlay(frame, center, marker_id, pos3d)

        # Status bar at the bottom
        n = len(ids) if ids is not None else 0
        status_color = (0, 200, 0) if self.intrinsics_ready else (0, 100, 255)
        status_text  = (
            f'Markers: {n}  |  Depth: {"OK" if self.last_depth_frame is not None else "waiting"}'
            f'  |  Intrinsics: {"OK" if self.intrinsics_ready else "waiting"}'
        )
        cv2.rectangle(frame, (0, frame.shape[0] - 28), (frame.shape[1], frame.shape[0]), (30, 30, 30), -1)
        cv2.putText(frame, status_text, (8, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1, cv2.LINE_AA)

        cv2.imshow('ArUco Tester', frame)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def deproject(self, pixel):
        """
        Return (X, Y, Z) in metres for a pixel centre, or None if unavailable.
        Uses median depth over a small window to reduce noise.
        """
        if not self.intrinsics_ready or self.last_depth_frame is None:
            return None

        x, y = int(pixel[0]), int(pixel[1])
        h, w = self.last_depth_frame.shape

        if not (0 <= x < w and 0 <= y < h):
            return None

        # Median over a 5×5 window to suppress noise / holes
        r = 2
        patch = self.last_depth_frame[
            max(0, y - r):min(h, y + r + 1),
            max(0, x - r):min(w, x + r + 1)
        ]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None

        depth_m = float(np.median(valid)) * self.depth_scale

        # Primary: RealSense SDK
        try:
            return rs.rs2_deproject_pixel_to_point(
                self.rs_intrinsics, [float(x), float(y)], depth_m
            )
        except Exception:
            pass

        # Fallback: pinhole model
        X = (x - self.ppx) * depth_m / self.fx
        Y = (y - self.ppy) * depth_m / self.fy
        return [X, Y, depth_m]

    def draw_overlay(self, frame, center, marker_id, pos3d):
        """Draw marker ID and 3D position above the marker centre."""
        cx, cy = center

        if pos3d is not None:
            x3, y3, z3 = pos3d
            lines = [
                f'ID: {marker_id}',
                f'X:{x3:+.3f}m',
                f'Y:{y3:+.3f}m',
                f'Z:{z3:.3f}m',
            ]
        else:
            lines = [f'ID: {marker_id}', 'depth: N/A']

        # Pill background then text
        line_h   = 18
        padding  = 4
        box_w    = 90
        box_h    = len(lines) * line_h + padding * 2
        box_x    = cx - box_w // 2
        box_y    = cy - box_h - 14  # sit just above the marker

        cv2.rectangle(frame,
                      (box_x, box_y),
                      (box_x + box_w, box_y + box_h),
                      (20, 20, 20), -1)
        cv2.rectangle(frame,
                      (box_x, box_y),
                      (box_x + box_w, box_y + box_h),
                      (0, 220, 0), 1)

        for j, line in enumerate(lines):
            color = (0, 255, 0) if j == 0 else (200, 255, 200)
            cv2.putText(frame, line,
                        (box_x + padding, box_y + padding + (j + 1) * line_h - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # Dot at marker centre
        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()