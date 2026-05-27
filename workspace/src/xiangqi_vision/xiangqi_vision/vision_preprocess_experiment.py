"""
Side-by-side preprocessing experiment node.

Runs board warp + YOLO on several presets in one tick and publishes a tiled
comparison on /xiangqi/preprocess_experiment_image. Use while tuning lab lighting
without changing the main vision_node pipeline.

Example (lab, with stack + camera running):

  ros2 run xiangqi_vision vision_preprocess_experiment --ros-args \\
    -p model_path:=/home/rosuser/workspace/models/xiangqi_kaggle_v5_best.pt \\
    -p calibration_file:=/home/rosuser/workspace/config/board_calibration.yaml \\
    -p compare_presets:="['none','clahe','lab_default','high_contrast']"

View: rqt_image_view /xiangqi/preprocess_experiment_image
"""

from __future__ import annotations

import ast
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .board_detector import BoardDetector, BoardCalibration
from .image_preprocess import PreprocessConfig, apply_piece_preprocess, stack_comparison
from .piece_detector import PieceDetector
from .weights_util import resolve_calibration_path, resolve_yolo_model_path

# Tiled output can be 2400x3500+; rqt often fails on huge messages. Match debug_image QoS too.
MAX_PUBLISH_WIDTH = 1600


def _parse_preset_list(value) -> list[str]:
    if isinstance(value, list):
        return [str(x) for x in value]
    if isinstance(value, str):
        text = value.strip()
        if text.startswith('['):
            try:
                parsed = ast.literal_eval(text)
                if isinstance(parsed, list):
                    return [str(x) for x in parsed]
            except (SyntaxError, ValueError):
                pass
        return [text] if text else ['none']
    return ['none']


def _fit_for_publish(image: np.ndarray, max_width: int) -> np.ndarray:
    h, w = image.shape[:2]
    if w <= max_width:
        return image
    scale = max_width / w
    return cv2.resize(
        image, (max_width, max(1, int(h * scale))), interpolation=cv2.INTER_AREA
    )


def _status_image(
    base: np.ndarray | None,
    title: str,
    detail: str,
    width: int = 800,
    height: int = 480,
) -> np.ndarray:
    if base is not None and base.size > 0:
        out = cv2.resize(base, (width, height), interpolation=cv2.INTER_AREA)
    else:
        out = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        out, title, (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA,
    )
    y = 80
    for line in detail.split('\n'):
        cv2.putText(
            out, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )
        y += 28
    return out


class VisionPreprocessExperiment(Node):
    def __init__(self) -> None:
        super().__init__('vision_preprocess_experiment')

        self.declare_parameter('model_path', '')
        self.declare_parameter('yolo_download_url', '')
        self.declare_parameter('calibration_file', '/home/rosuser/workspace/config/board_calibration.yaml')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'compare_presets',
            ['none', 'bright', 'bright_sharp'],
        )
        self.declare_parameter('publish_rate_hz', 0.2)
        self.declare_parameter('max_publish_width', MAX_PUBLISH_WIDTH)

        cal_file = resolve_calibration_path(
            self.get_parameter('calibration_file').value, self.get_logger()
        )
        cal = BoardCalibration()
        if os.path.exists(cal_file):
            cal = BoardCalibration.load(cal_file)
            self.get_logger().info(f'Loaded calibration from {cal_file}')

        self._board_detector = BoardDetector(cal)
        model_path = resolve_yolo_model_path(
            str(self.get_parameter('model_path').value or ''),
            param_download_url=str(self.get_parameter('yolo_download_url').value or ''),
            logger=self.get_logger(),
        )
        if not model_path:
            raise RuntimeError('YOLO weights required for preprocess experiment')
        self._piece_detector = PieceDetector(
            model_path, float(self.get_parameter('confidence_threshold').value)
        )
        self._presets = _parse_preset_list(self.get_parameter('compare_presets').value)
        self._max_width = int(self.get_parameter('max_publish_width').value)
        self._bridge = CvBridge()
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._frames_in = 0
        self._publishes_out = 0
        self._processing = False

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cam_topic = self.get_parameter('camera_topic').value
        self.create_subscription(Image, cam_topic, self._image_callback, camera_qos)
        self._pub = self.create_publisher(
            Image, '/xiangqi/preprocess_experiment_image', sensor_qos
        )
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._publish_tick)
        self.get_logger().info(
            f'Presets {self._presets} -> /xiangqi/preprocess_experiment_image '
            f'(camera={cam_topic}, ~{len(self._presets)} YOLO runs/tick, BEST_EFFORT QoS)'
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            with self._lock:
                self._latest = img
                self._frames_in += 1
                if self._frames_in == 1:
                    self.get_logger().info(
                        f'Camera OK ({img.shape[1]}x{img.shape[0]}) from subscription'
                    )
        except Exception as e:
            self.get_logger().error(f'Image conversion: {e}')

    def _emit(self, image: np.ndarray) -> None:
        image = np.ascontiguousarray(_fit_for_publish(image, self._max_width))
        msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self._pub.publish(msg)
        self._publishes_out += 1
        self.get_logger().info(
            f'Published experiment frame #{self._publishes_out} ({image.shape[1]}x{image.shape[0]})',
            throttle_duration_sec=5.0,
        )

    def _publish_tick(self) -> None:
        if self._processing:
            return
        with self._lock:
            image = None if self._latest is None else self._latest.copy()
        if image is None:
            try:
                self._emit(_status_image(
                    None,
                    'No camera frames',
                    f'Subscribe to:\n{self.get_parameter("camera_topic").value}\n'
                    'Start RealSense / vision stack first.',
                ))
            except Exception as e:
                self.get_logger().error(f'Status publish failed: {e}', throttle_duration_sec=5.0)
            self.get_logger().warn('Waiting for camera frames...', throttle_duration_sec=5.0)
            return

        self._processing = True
        try:
            self._run_comparison(image)
        except Exception as e:
            self.get_logger().error(f'Comparison failed: {e}')
            try:
                self._emit(_status_image(image, 'Experiment error', str(e)))
            except Exception:
                pass
        finally:
            self._processing = False

    def _run_comparison(self, image: np.ndarray) -> None:
        t0 = time.monotonic()
        ok, H, aruco_debug = self._board_detector.detect(image)
        if not ok:
            self.get_logger().warn('ArUco failed - publishing camera + marker debug', throttle_duration_sec=5.0)
            self._emit(_status_image(
                aruco_debug,
                'ArUco failed (need IDs 0-3)',
                getattr(self._board_detector, '_last_detect_diag', ''),
            ))
            return

        warped = self._board_detector.warp_board(image, H)
        rows = []
        n_presets = len(self._presets)
        self.get_logger().info(
            f'Running {n_presets} preset(s) (CPU may take {n_presets * 2}-{n_presets * 8}s)...',
            throttle_duration_sec=10.0,
        )

        for i, preset in enumerate(self._presets):
            cfg = PreprocessConfig.from_preset(str(preset), enabled=(preset != 'none'))
            processed = apply_piece_preprocess(warped, cfg)
            detections, annotated = self._piece_detector.detect(processed)
            n_pieces = len(detections)
            mean_conf = self._piece_detector.mean_confidence(detections)
            label = f'{preset} ({n_pieces} pcs, conf={mean_conf:.2f})'
            row = stack_comparison(warped, processed, annotated, ('warp', preset, 'yolo'))
            cv2.putText(
                row, label, (8, row.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA,
            )
            rows.append(row)

        tile_w = rows[0].shape[1]
        tile_h = rows[0].shape[0]
        canvas = np.zeros((tile_h * len(rows), tile_w, 3), dtype=np.uint8)
        for i, row in enumerate(rows):
            canvas[i * tile_h:(i + 1) * tile_h, :tile_w] = row

        elapsed = time.monotonic() - t0
        cv2.putText(
            canvas, f'{elapsed:.1f}s', (tile_w - 120, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )
        self._emit(canvas)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPreprocessExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
