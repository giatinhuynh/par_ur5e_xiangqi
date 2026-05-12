"""
calibration_tool: Interactive utility to calibrate the board-to-robot transform.

Usage:
  ros2 run xiangqi_vision calibration_tool

Steps:
  1. Place the board under the camera. The tool will show the camera feed.
     Ensure all 4 ArUco markers are visible. Press SPACE to capture.
  2. Using the teach pendant, move the robot TCP to 4 reference points
     (the 4 board corners), recording the TCP pose after each.
  3. The tool computes the board_to_base_tf and saves to calibration.yaml.
"""

import os
import sys
import time
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from .board_detector import BoardDetector, BoardCalibration, BOARD_FILES, BOARD_RANKS


CALIBRATION_OUTPUT = '/home/rosuser/workspace/config/board_calibration.yaml'

# The 4 corner grid positions we use for the robot teach-in
# (file, rank) -> description
CALIBRATION_CORNERS = [
    (0, 0),   # Bottom-left  (red side, file a, rank 0)
    (8, 0),   # Bottom-right (red side, file i, rank 0)
    (8, 9),   # Top-right    (black side, file i, rank 9)
    (0, 9),   # Top-left     (black side, file a, rank 9)
]


class CalibrationTool(Node):
    def __init__(self):
        super().__init__('calibration_tool')
        self._bridge = CvBridge()
        self._detector = BoardDetector()
        self._calibration = BoardCalibration()
        self._latest_image = None
        self._captured_homography = None
        self._tcp_poses = []  # List of recorded TCP positions

        self._img_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._img_cb, 1
        )
        self._tcp_sub = self.create_subscription(
            PoseStamped, '/tool_pose', self._tcp_cb, 1
        )

        self.get_logger().info('Calibration tool started. Press SPACE in the window to capture board.')

    def _img_cb(self, msg):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _tcp_cb(self, msg: PoseStamped):
        self._latest_tcp = msg.pose

    def run(self):
        cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)
        self.get_logger().info('Step 1: Show board with ArUco markers. Press SPACE to capture.')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_image is None:
                continue

            img = self._latest_image.copy()
            ok, H, debug = self._detector.detect(img)
            status = 'BOARD DETECTED' if ok else 'BOARD NOT FOUND'
            colour = (0, 255, 0) if ok else (0, 0, 255)
            cv2.putText(debug, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)
            cv2.imshow('Calibration', debug)

            key = cv2.waitKey(30) & 0xFF
            if key == ord(' ') and ok:
                self._captured_homography = H
                self._calibration.homography = H
                self.get_logger().info('Homography captured!')
                break
            elif key == ord('q'):
                return

        cv2.destroyAllWindows()

        # Step 2: Record robot TCP at 4 corners
        print('\nStep 2: Record robot TCP at board corners.')
        print('Move the robot arm so the TCP touches each corner intersection listed below.')
        print('Press ENTER after positioning at each corner.\n')

        for i, (file_idx, rank_idx) in enumerate(CALIBRATION_CORNERS):
            print(f'  Corner {i+1}/4: file={file_idx} ("{"abcdefghi"[file_idx]}"), rank={rank_idx}')
            input('  >>> Press ENTER when TCP is at this corner... ')
            rclpy.spin_once(self, timeout_sec=0.1)
            if hasattr(self, '_latest_tcp'):
                p = self._latest_tcp.position
                tcp = np.array([p.x, p.y, p.z])
                self._tcp_poses.append(tcp)
                print(f'  Recorded: {tcp}')
            else:
                # Manual entry fallback
                vals = input('  Enter TCP x y z (metres, space-separated): ').split()
                self._tcp_poses.append(np.array([float(v) for v in vals]))

        # Step 3: Compute board_to_base_tf
        self._compute_transform()
        self._calibration.save(CALIBRATION_OUTPUT)
        self.get_logger().info(f'Calibration saved to {CALIBRATION_OUTPUT}')
        print(f'\nCalibration saved to {CALIBRATION_OUTPUT}')

    def _compute_transform(self):
        """
        Compute the 4x4 board_frame -> robot_base rigid transform.
        board_frame origin = (file=0, rank=0), X = file direction, Y = rank direction.
        """
        spacing = self._calibration.grid_spacing_mm / 1000.0

        # Board-frame positions of the 4 calibration corners
        board_pts = np.array([
            [f * spacing, r * spacing, 0.0]
            for (f, r) in CALIBRATION_CORNERS
        ], dtype=float)

        # Robot-base positions recorded at those corners
        robot_pts = np.array(self._tcp_poses, dtype=float)

        # Solve: robot_pt = R @ board_pt + t  (rigid body, 3 DOF rotation + 3 DOF translation)
        # Use least squares on the centred point clouds
        board_centroid = board_pts.mean(axis=0)
        robot_centroid = robot_pts.mean(axis=0)
        B = board_pts - board_centroid
        R_pts = robot_pts - robot_centroid
        H = B.T @ R_pts
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = robot_centroid - R @ board_centroid

        tf = np.eye(4)
        tf[:3, :3] = R
        tf[:3, 3] = t
        self._calibration.board_to_base_tf = tf
        self._calibration.board_origin_mm = (0.0, 0.0)
        self._calibration.grid_spacing_mm = 45.0

        print(f'\nComputed board_to_base_tf:\n{tf}')
        residuals = np.linalg.norm(
            robot_pts - (R @ board_pts.T).T - t, axis=1
        )
        print(f'Residuals (mm): {residuals * 1000}')


def main(args=None):
    rclpy.init(args=args)
    tool = CalibrationTool()
    tool.run()
    tool.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
