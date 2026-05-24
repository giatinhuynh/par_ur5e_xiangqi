"""
vision_node: Main ROS 2 node for the Reactive (Tier 1) vision layer.

Responsibilities:
  - Subscribes to the RealSense RGB camera topic
  - Runs ArUco board detection and YOLOv8 piece detection
  - Publishes BoardState on /xiangqi/board_state
  - Runs turn detection; publishes on /xiangqi/human_move_detected
  - Provides GetBoardState service
  - Accepts keyboard fallback on /xiangqi/human_ready

Architecture (threading):
  Camera callback  →  _frame_queue (maxsize=1, always latest frame)
  _run_detection_loop (daemon thread)  ←  consumes queue, runs ArUco+YOLO, writes _pending_*
  _detection_tick (timer, executor thread)  →  reads _pending_*, publishes to ROS topics

  The camera callback and timer callback are both trivially fast — no blocking anywhere on the
  executor thread. Heavy computation is entirely in the dedicated detection thread.
"""

import os
import queue
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
        self._latest_board_state: BoardState | None = None
        self._lock = threading.Lock()

        # Frame queue: camera callback always puts the latest frame here (maxsize=1 = always fresh).
        # Detection thread consumes it. No _processing flag needed.
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)

        # Results written by detection thread, read+published by timer on the executor thread
        self._pending_board_state: BoardState | None = None
        self._pending_debug_image: np.ndarray | None = None
        self._pending_move_detected: bool = False
        self._debug_frames_published: int = 0
        self._camera_frames_received: int = 0
        self._detection_runs: int = 0

        # --- QoS ---
        # Camera publishes RELIABLE — subscription must match or Fast DDS stops delivering
        # after a few frames despite showing the subscription as connected.
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

        # --- Subscriptions ---
        self._img_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self._image_callback,
            camera_qos,
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
        # BEST_EFFORT so publish() is always non-blocking (RELIABLE can stall on large images)
        self._debug_img_pub = self.create_publisher(Image, '/xiangqi/debug_image', sensor_qos)

        # --- Services ---
        self._get_board_srv = self.create_service(
            GetBoardState, 'get_board_state', self._get_board_state_callback
        )

        # --- Dedicated detection thread (permanent daemon) ---
        self._detection_thread = threading.Thread(
            target=self._run_detection_loop, daemon=True, name='vision_detection'
        )
        self._detection_thread.start()

        # --- Publish timer (executor thread only — no heavy work here) ---
        period = 1.0 / self._poll_rate
        self._timer = self.create_timer(period, self._publish_tick)

        self.get_logger().info('vision_node started')

    # ------------------------------------------------------------------
    # Camera callback — must be trivially fast, no blocking
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        try:
            # .copy() guarantees we own the buffer (cv_bridge may return a view into DDS memory)
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
            self._camera_frames_received += 1
            self.get_logger().info(
                f'Camera frame #{self._camera_frames_received} received '
                f'({img.shape[1]}x{img.shape[0]})',
                throttle_duration_sec=10.0,
            )
            # Replace any unconsumed frame in the queue with this latest one
            try:
                self._frame_queue.put_nowait(img)
            except queue.Full:
                try:
                    self._frame_queue.get_nowait()   # discard stale frame
                except queue.Empty:
                    pass
                self._frame_queue.put_nowait(img)    # put latest
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    # ------------------------------------------------------------------
    # Other callbacks — all fast, no YOLO/ArUco here
    # ------------------------------------------------------------------

    def _human_ready_callback(self, _: Empty) -> None:
        self.get_logger().info('Keyboard fallback triggered -- forcing move detection')
        self._turn_detector.trigger_keyboard_fallback()

    def _start_watching_callback(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                state = self._latest_board_state
            if state is not None:
                self._turn_detector.start_watching(np.array(state.grid, dtype=np.int8))
                self.get_logger().info('Turn detection: now watching for human move')
            else:
                self.get_logger().warn('start_watching: no board state yet — using empty baseline')
                self._turn_detector.start_watching(np.zeros(90, dtype=np.int8))
        else:
            self._turn_detector.stop_watching()

    def _get_board_state_callback(self, request, response):
        # Non-blocking — always returns the cached result from the detection thread
        with self._lock:
            state = self._latest_board_state
        if state is not None:
            response.board_state = state
            response.success = True
            response.message = 'OK'
        else:
            response.success = False
            response.message = 'No board detected yet'
        return response

    # ------------------------------------------------------------------
    # Publish tick — runs on executor thread, trivially fast
    # ------------------------------------------------------------------

    def _publish_tick(self) -> None:
        with self._lock:
            board_state = self._pending_board_state
            debug_img   = self._pending_debug_image
            move_det    = self._pending_move_detected
            self._pending_board_state  = None
            self._pending_debug_image  = None
            self._pending_move_detected = False

        if debug_img is not None:
            self._publish_debug(debug_img)
            self._debug_frames_published += 1
            self.get_logger().info(
                f'Debug frame published #{self._debug_frames_published} '
                f'(shape={debug_img.shape}, dtype={debug_img.dtype})',
                throttle_duration_sec=10.0,
            )
        if board_state is not None:
            self._board_state_pub.publish(board_state)
        if move_det:
            self.get_logger().info('Human move detected and confirmed by vision stability')
            msg = Bool()
            msg.data = True
            self._move_detected_pub.publish(msg)

    # ------------------------------------------------------------------
    # Dedicated detection loop — permanent daemon thread, never on executor
    # ------------------------------------------------------------------

    def _run_detection_loop(self) -> None:
        """Runs forever in its own thread. Blocks on _frame_queue, processes latest frame."""
        self.get_logger().info('Detection loop thread started')
        while rclpy.ok():
            try:
                image = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            self._detection_runs += 1
            run_id = self._detection_runs
            t0 = time.monotonic()
            try:
                board_state, debug_img, move_det = self._process_frame(image)
                elapsed = time.monotonic() - t0
                self.get_logger().info(
                    f'Detection #{run_id}: {elapsed:.2f}s  '
                    f'board={board_state is not None}  '
                    f'img_shape={image.shape}',
                    throttle_duration_sec=10.0,
                )
                with self._lock:
                    if board_state is not None:
                        self._latest_board_state = board_state
                    self._pending_board_state  = board_state
                    self._pending_debug_image  = debug_img
                    self._pending_move_detected = move_det
            except Exception as e:
                self.get_logger().error(f'Detection #{run_id} exception: {e}')

        self.get_logger().info('Detection loop thread exiting')

    # ------------------------------------------------------------------
    # Core frame processing — pure computation, called only from detection thread
    # ------------------------------------------------------------------

    def _process_frame(
        self, image: np.ndarray
    ) -> tuple[BoardState | None, np.ndarray | None, bool]:
        ok, H, debug = self._board_detector.detect(image)

        if not ok:
            diag = getattr(self._board_detector, '_last_detect_diag', '')
            self.get_logger().warn(
                f'ArUco detection failed -- board not visible. {diag.replace(chr(10), " ")}',
                throttle_duration_sec=5.0,
            )
            return None, debug, False

        warped = self._board_detector.warp_board(image, H)

        grid = np.zeros(90, dtype=np.int8)
        mean_conf = 0.0

        if self._piece_detector is not None:
            detections, annotated = self._piece_detector.detect(warped)
            grid = self._piece_detector.detections_to_grid(detections)
            mean_conf = self._piece_detector.mean_confidence(detections)
            debug_out = annotated
        else:
            debug_out = warped

        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.grid = grid.tolist()
        msg.detection_confidence = mean_conf

        grid_arr = np.array(msg.grid, dtype=np.int8)
        _, confirmed = self._turn_detector.update(grid_arr)
        move_detected = bool(confirmed)

        return msg, debug_out, move_detected

    # ------------------------------------------------------------------
    # Debug image publishing — called from executor thread (_publish_tick)
    # ------------------------------------------------------------------

    def _publish_debug(self, image: np.ndarray) -> None:
        try:
            if image.dtype != np.uint8:
                image = np.clip(image * 255, 0, 255).astype(np.uint8)
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif image.shape[2] == 4:
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            image = np.ascontiguousarray(image).copy()
            cv2.putText(
                image, f'#{self._debug_frames_published}',
                (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3, cv2.LINE_AA,
            )
            cv2.putText(
                image, f'#{self._debug_frames_published}',
                (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA,
            )
            msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self._debug_img_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'_publish_debug failed: {e}', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)   # SingleThreadedExecutor — all callbacks are fast, no blocking
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
