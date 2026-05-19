"""
vision_node: Main ROS 2 node for the Reactive (Tier 1) vision layer.

Responsibilities:
  - Subscribes to the RealSense RGB camera topic
  - Runs ArUco board detection and YOLOv8 piece detection
  - Publishes BoardState on /xiangqi/board_state
  - Runs turn detection; publishes on /xiangqi/human_move_detected
  - Provides GetBoardState service
  - Accepts keyboard fallback on /xiangqi/human_ready
"""

import os
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Header, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from xiangqi_msgs.msg import BoardState, PieceDetection
from xiangqi_msgs.srv import GetBoardState

from .board_detector import BoardDetector, BoardCalibration
from .piece_detector import PieceDetector
from .turn_detector import TurnDetector, TurnDetectorState
from .weights_util import resolve_calibration_path, resolve_yolo_model_path


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '/home/rosuser/workspace/models/xiangqi_kaggle_v1_best.pt')
        self.declare_parameter('calibration_file', '/home/rosuser/workspace/config/board_calibration.yaml')
        self.declare_parameter('require_yolo_weights', True)
        self.declare_parameter('yolo_download_url', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('stability_frames', 8)
        self.declare_parameter('poll_rate_hz', 3.0)
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')

        model_path_param = self.get_parameter('model_path').value
        cal_file = resolve_calibration_path(self.get_parameter('calibration_file').value, self.get_logger())
        conf_thresh = self.get_parameter('confidence_threshold').value
        stability = self.get_parameter('stability_frames').value
        self._poll_rate = self.get_parameter('poll_rate_hz').value
        require_yolo = bool(self.get_parameter('require_yolo_weights').value)
        yolo_url = str(self.get_parameter('yolo_download_url').value or '')

        # --- Calibration ---
        self._calibration = BoardCalibration()
        if os.path.exists(cal_file):
            self._calibration = BoardCalibration.load(cal_file)
            self.get_logger().info(f'Loaded calibration from {cal_file}')
        else:
            self.get_logger().warn(f'No calibration file at {cal_file} -- ArUco runtime detection only')

        # --- Components ---
        self._board_detector = BoardDetector(self._calibration)
        self._piece_detector: PieceDetector | None = None
        resolved_model = resolve_yolo_model_path(
            model_path_param,
            param_download_url=yolo_url,
            logger=self.get_logger(),
        )
        if resolved_model:
            try:
                self._piece_detector = PieceDetector(resolved_model, conf_thresh)
                self.get_logger().info(f'YOLOv8 model loaded from {resolved_model}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                if require_yolo:
                    raise RuntimeError(
                        'require_yolo_weights is true but YOLO failed to load. '
                        'Place a compatible .pt under xiangqi_vision/share/.../models/, '
                        'set model_path, or set yolo_download_url / XIANGQI_YOLO_DOWNLOAD_URL.'
                    ) from e
        elif require_yolo:
            raise RuntimeError(
                'require_yolo_weights is true but no weights file was found. '
                'Install xiangqi_kaggle_v1_best.pt into share/xiangqi_vision/models/, '
                'point model_path at your .pt, or set yolo_download_url / XIANGQI_YOLO_DOWNLOAD_URL.'
            )
        else:
            self.get_logger().warn(
                'YOLO weights not found — piece detection disabled (require_yolo_weights:=false)'
            )

        self._turn_detector = TurnDetector(stability_frames=stability)
        self._bridge = CvBridge()

        # --- State ---
        self._latest_image: np.ndarray | None = None
        self._latest_board_state: BoardState | None = None
        self._current_homography: np.ndarray | None = None
        self._lock = threading.Lock()

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscriptions ---
        self._img_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self._image_callback,
            sensor_qos,
        )
        self._human_ready_sub = self.create_subscription(
            Empty,
            '/xiangqi/human_ready',
            self._human_ready_callback,
            10,
        )
        self._watch_sub = self.create_subscription(
            Bool,
            '/xiangqi/start_watching',
            self._start_watching_callback,
            10,
        )

        # --- Publishers ---
        self._board_state_pub = self.create_publisher(BoardState, '/xiangqi/board_state', 10)
        self._move_detected_pub = self.create_publisher(Bool, '/xiangqi/human_move_detected', 10)
        self._debug_img_pub = self.create_publisher(Image, '/xiangqi/debug_image', sensor_qos)

        # --- Services ---
        self._get_board_srv = self.create_service(
            GetBoardState, 'get_board_state', self._get_board_state_callback
        )

        # --- Detection timer ---
        period = 1.0 / self._poll_rate
        self._timer = self.create_timer(period, self._detection_tick)

        self.get_logger().info('vision_node started')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._latest_image = img
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def _human_ready_callback(self, _: Empty) -> None:
        self.get_logger().info('Keyboard fallback triggered -- forcing move detection')
        self._turn_detector.trigger_keyboard_fallback()

    def _start_watching_callback(self, msg: Bool) -> None:
        if msg.data:
            current_state = self._get_current_board_state()
            if current_state is not None:
                self._turn_detector.start_watching(np.array(current_state.grid, dtype=np.int8))
                self.get_logger().info('Turn detection: now watching for human move')
        else:
            self._turn_detector.stop_watching()

    def _get_board_state_callback(self, request, response):
        if request.force_rescan or self._latest_board_state is None:
            state = self._get_current_board_state()
        else:
            state = self._latest_board_state

        if state is not None:
            response.board_state = state
            response.success = True
            response.message = 'OK'
        else:
            response.success = False
            response.message = 'No board detected'
        return response

    # ------------------------------------------------------------------
    # Detection tick
    # ------------------------------------------------------------------

    def _detection_tick(self) -> None:
        with self._lock:
            if self._latest_image is None:
                return
            image = self._latest_image.copy()

        board_state = self._process_image(image)
        if board_state is None:
            return

        with self._lock:
            self._latest_board_state = board_state

        self._board_state_pub.publish(board_state)

        # Turn detection update
        grid = np.array(board_state.grid, dtype=np.int8)
        confirmed, confirmed_grid = self._turn_detector.update(grid)
        if confirmed:
            self.get_logger().info('Human move detected and confirmed by vision stability')
            msg = Bool()
            msg.data = True
            self._move_detected_pub.publish(msg)

    # ------------------------------------------------------------------
    # Core processing
    # ------------------------------------------------------------------

    def _process_image(self, image: np.ndarray) -> BoardState | None:
        ok, H, debug = self._board_detector.detect(image)

        if not ok:
            diag = getattr(self._board_detector, '_last_detect_diag', '')
            self.get_logger().warn(
                f'ArUco detection failed -- board not visible. {diag.replace(chr(10), " ")}',
                throttle_duration_sec=5.0,
            )
            self._publish_debug(debug)
            return None

        self._current_homography = H
        warped = self._board_detector.warp_board(image, H)

        grid = np.zeros(90, dtype=np.int8)
        mean_conf = 0.0

        if self._piece_detector is not None:
            detections, annotated = self._piece_detector.detect(warped)
            grid = self._piece_detector.detections_to_grid(detections)
            mean_conf = self._piece_detector.mean_confidence(detections)
            self._publish_debug(annotated)
        else:
            self._publish_debug(warped)

        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.grid = grid.tolist()
        msg.detection_confidence = mean_conf
        return msg

    def _get_current_board_state(self) -> BoardState | None:
        with self._lock:
            if self._latest_image is None:
                return None
            image = self._latest_image.copy()
        return self._process_image(image)

    def _publish_debug(self, image: np.ndarray) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self._debug_img_pub.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
