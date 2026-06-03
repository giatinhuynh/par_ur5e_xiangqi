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

  The camera callback and timer callback are both trivially fast - no blocking anywhere on the
  executor thread. Heavy computation is entirely in the dedicated detection thread.
"""

import os
import queue
import time
import threading
import numpy as np
import cv2
import yaml as _yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration as RclpyDuration
from std_msgs.msg import Bool, Header, Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

from xiangqi_msgs.msg import BoardState, PieceDetection
from xiangqi_vision.fen_util import grid_to_fen
from xiangqi_msgs.srv import GetBoardState, GetBoardTransform

from .board_detector import BoardDetector, BoardCalibration
from .board_layout import _compute_4xa3_grid_mm as _blay_compute
from .image_preprocess import PreprocessConfig, apply_piece_preprocess, stack_comparison
from .piece_detector import PieceDetector
from .turn_detector import TurnDetector, TurnDetectorState
from .weights_util import resolve_calibration_path, resolve_yolo_model_path


def _tf_rotation_matrix(r) -> np.ndarray:
    """Convert a geometry_msgs Quaternion to a 3×3 rotation matrix."""
    x, y, z, w = r.x, r.y, r.z, r.w
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _apply_transform(tf_stamped, point_cam: np.ndarray) -> np.ndarray:
    """Transform a 3D point from camera frame to target frame via a TF stamped."""
    R = _tf_rotation_matrix(tf_stamped.transform.rotation)
    t = tf_stamped.transform.translation
    return R @ point_cam + np.array([t.x, t.y, t.z])


def _compute_4xa3_marker_fractions():
    """Compute bilinear (u,v) fractions for grid corners within the marker quad.

    Returns (u_file0, u_file8, v_rank0, v_rank9) using the physical marker centre
    positions (not the norm_pixel inner-band fractions used for YOLO snapping).
    """
    gx, gy, il, *_ = _blay_compute()
    page_w = 594.0
    page_h = 840.0
    aruco_page_inset = 8.0
    aruco_size = 38.0
    marker_inset = aruco_page_inset + aruco_size / 2.0  # 27mm from sheet corner
    marker_span_x = page_w - 2.0 * marker_inset          # 540mm
    marker_span_y = page_h - 2.0 * marker_inset          # 786mm

    delta_x = il - marker_inset                          # inner_left - 27 = 25mm
    delta_y = (page_h - gy(0)) - marker_inset            # (840 - 695.625) - 27 = 117.375mm

    cell_mm = gx(1) - gx(0)
    u_file0 = delta_x / marker_span_x
    u_file8 = (delta_x + 8.0 * cell_mm) / marker_span_x
    v_rank0 = delta_y / marker_span_y
    v_rank9 = (delta_y + 9.0 * cell_mm) / marker_span_y
    return u_file0, u_file8, v_rank0, v_rank9


_MAT_U_FILE0, _MAT_U_FILE8, _MAT_V_RANK0, _MAT_V_RANK9 = _compute_4xa3_marker_fractions()

# ArUco marker-centre positions in the board frame (origin=a0, X=file dir, Y=rank dir, metres).
# Derived from 4×A3 mat geometry; used for auto camera calibration.
_CELL_M = 0.06125   # grid spacing
_DX_M   = 0.025     # inner_left (52mm) - marker_inset (27mm)
_DY_M   = 0.117375  # (page_h - bb_grid) - marker_inset = 144.375 - 27
_MARKER_BOARD_M: dict = {
    0: np.array([-_DX_M,         9 * _CELL_M + _DY_M, 0.0]),  # ID0: file0, rank9-side
    1: np.array([8 * _CELL_M + _DX_M, 9 * _CELL_M + _DY_M, 0.0]),  # ID1: file8, rank9-side
    2: np.array([8 * _CELL_M + _DX_M, -_DY_M,              0.0]),  # ID2: file8, rank0-side
    3: np.array([-_DX_M,         -_DY_M,                   0.0]),  # ID3: file0, rank0-side
}


def _kabsch_tf(pts_from: np.ndarray, pts_to: np.ndarray) -> np.ndarray:
    """Kabsch/SVD rigid-body alignment: find 4×4 T so pts_to ≈ T[:3,:3] @ pts_from + T[:3,3].

    pts_from / pts_to: (N, 3) arrays of corresponding 3D points.
    """
    c_f = pts_from.mean(axis=0)
    c_t = pts_to.mean(axis=0)
    H   = (pts_from - c_f).T @ (pts_to - c_t)
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = c_t - R @ c_f
    tf       = np.eye(4)
    tf[:3, :3] = R
    tf[:3,  3] = t
    return tf


def _rotation_to_quaternion(R: np.ndarray):
    """Convert 3×3 rotation matrix to (x, y, z, w) quaternion tuple."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s


def _compute_board_tf_from_markers(markers_base: dict) -> np.ndarray:
    """Compute 4×4 board_to_base_tf from ArUco marker centre 3D positions.

    markers_base: {id: np.array([x,y,z])} for IDs 0-3 in base_link frame.
    Board frame: origin=a0 (file0,rank0), X=file dir (a→i), Y=rank dir (0→9).
    """
    M0, M1, M2, M3 = [np.array(markers_base[i]) for i in range(4)]

    def _bilin(u, v):
        return (1-u)*(1-v)*M3 + u*(1-v)*M2 + u*v*M1 + (1-u)*v*M0

    a0 = _bilin(_MAT_U_FILE0, _MAT_V_RANK0)
    i0 = _bilin(_MAT_U_FILE8, _MAT_V_RANK0)
    a9 = _bilin(_MAT_U_FILE0, _MAT_V_RANK9)

    X_hat = i0 - a0
    X_hat /= np.linalg.norm(X_hat)
    Z_hat = np.cross(X_hat, a9 - a0)
    Z_hat /= np.linalg.norm(Z_hat)
    Y_hat = np.cross(Z_hat, X_hat)

    tf = np.eye(4)
    tf[:3, 0] = X_hat
    tf[:3, 1] = Y_hat
    tf[:3, 2] = Z_hat
    tf[:3, 3] = a0
    return tf


class GridStabilizer:
    """
    Per-cell temporal smoothing for the raw YOLO detection grid.

    Each of the 90 board cells only changes its committed value after the
    *same* value has been observed in `smooth_frames` consecutive raw
    detection frames.  A single flickering frame is silently ignored;
    genuine piece placements/removals are committed after a short delay
    (~smooth_frames / poll_rate_hz seconds).

    With smooth_frames=3 at 3 Hz the delay is ~1 s - short enough to feel
    immediate to the human, long enough to absorb YOLO glitches.
    Setting smooth_frames=1 disables smoothing entirely.
    """

    def __init__(self, smooth_frames: int = 3, n_cells: int = 90):
        self._n = max(1, smooth_frames)
        # Last committed (output) grid - starts all-empty
        self._committed = np.zeros(n_cells, dtype=np.int8)
        # Candidate value per cell (what we are counting towards)
        self._candidate = np.zeros(n_cells, dtype=np.int8)
        # Consecutive-frame streak per cell for the current candidate
        self._streak = np.zeros(n_cells, dtype=np.int32)

    def update(self, raw: np.ndarray) -> np.ndarray:
        """Feed a raw detection grid; return the smoothed committed grid."""
        same = raw == self._candidate
        self._streak[same] += 1
        # New value for a cell → restart candidate and streak
        changed = ~same
        self._candidate[changed] = raw[changed]
        self._streak[changed] = 1
        # Commit cells whose streak has reached the threshold
        ready = self._streak >= self._n
        self._committed[ready] = self._candidate[ready]
        return self._committed.copy()

    def reset(self) -> None:
        """Clear all history (call when the game resets or camera is repositioned)."""
        self._committed[:] = 0
        self._candidate[:] = 0
        self._streak[:] = 0


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '/home/rosuser/workspace/models/xiangqi_kaggle_v4_best.pt')
        self.declare_parameter('calibration_file', '/home/rosuser/workspace/config/board_calibration.yaml')
        self.declare_parameter('require_yolo_weights', True)
        self.declare_parameter('yolo_download_url', '')
        self.declare_parameter('confidence_threshold', 0.5)
        # 0 = use imgsz from checkpoint train_args (v4 → 640)
        self.declare_parameter('yolo_imgsz', 0)
        self.declare_parameter('stability_frames', 8)
        self.declare_parameter('grid_smooth_frames', 3)
        self.declare_parameter('poll_rate_hz', 3.0)
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        # Piece detection preprocessing (warped board, before YOLO)
        self.declare_parameter('piece_preprocess_enabled', False)
        self.declare_parameter('piece_preprocess_preset', 'none')
        self.declare_parameter('piece_gamma', 1.0)
        self.declare_parameter('piece_brightness', 0)
        self.declare_parameter('piece_contrast', 1.0)
        self.declare_parameter('piece_use_clahe', False)
        self.declare_parameter('piece_clahe_clip_limit', 2.0)
        self.declare_parameter('piece_use_denoise', False)
        self.declare_parameter('piece_use_sharpen', False)
        self.declare_parameter('piece_saturation_scale', 1.0)
        self.declare_parameter('piece_use_white_balance', False)
        self.declare_parameter('debug_show_preprocess', False)
        self.declare_parameter(
            'camera_config_file',
            '/home/rosuser/workspace/config/camera_config.yaml',
        )

        model_path_param = self.get_parameter('model_path').value
        cal_file = resolve_calibration_path(self.get_parameter('calibration_file').value, self.get_logger())
        conf_thresh = self.get_parameter('confidence_threshold').value
        yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        stability = self.get_parameter('stability_frames').value
        grid_smooth = int(self.get_parameter('grid_smooth_frames').value)
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
                self._piece_detector = PieceDetector(
                    resolved_model, conf_thresh, yolo_imgsz=yolo_imgsz,
                )
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
                'Install xiangqi_kaggle_v4_best.pt into workspace/models/ or share/xiangqi_vision/models/, '
                'point model_path at your .pt, or set yolo_download_url / XIANGQI_YOLO_DOWNLOAD_URL.'
            )
        else:
            self.get_logger().warn(
                'YOLO weights not found - piece detection disabled (require_yolo_weights:=false)'
            )

        self._turn_detector = TurnDetector(stability_frames=stability)
        self._grid_stabilizer = GridStabilizer(smooth_frames=grid_smooth)
        self._bridge = CvBridge()

        # --- TF2 for camera→base_link transform (fallback if no auto-calibration) ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Static TF broadcaster: used to republish auto-calibrated camera TF ---
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # --- Camera intrinsics (from CameraInfo) ---
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None

        # --- Camera→base_link transform (auto-calibrated or loaded from file) ---
        self._camera_to_base_matrix: np.ndarray | None = None
        self._load_camera_tf()  # try loading saved calibration right away

        # --- State ---
        self._latest_board_state: BoardState | None = None
        self._last_camera_image: np.ndarray | None = None
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
        # Camera publishes RELIABLE - subscription must match or Fast DDS stops delivering
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

        cam_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
        camera_info_topic = self.get_parameter('camera_topic').value.replace(
            'image_raw', 'camera_info'
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            cam_info_qos,
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
        self._get_board_transform_srv = self.create_service(
            GetBoardTransform, 'get_board_transform', self._get_board_transform_callback
        )

        # --- Dedicated detection thread (permanent daemon) ---
        self._detection_thread = threading.Thread(
            target=self._run_detection_loop, daemon=True, name='vision_detection'
        )
        self._detection_thread.start()

        # --- Publish timer (executor thread only - no heavy work here) ---
        period = 1.0 / self._poll_rate
        self._timer = self.create_timer(period, self._publish_tick)

        self.get_logger().info('vision_node started')

    def _piece_preprocess_config(self) -> PreprocessConfig:
        """Read preprocess params (safe to call each detection tick for live tuning)."""
        defaults = PreprocessConfig()
        enabled = bool(self.get_parameter('piece_preprocess_enabled').value)
        preset = str(self.get_parameter('piece_preprocess_preset').value)
        if enabled and preset != 'none':
            cfg = PreprocessConfig.from_preset(preset, enabled=True)
        else:
            cfg = PreprocessConfig(enabled=enabled, preset=preset)

        overrides = {
            'gamma': float(self.get_parameter('piece_gamma').value),
            'brightness': int(self.get_parameter('piece_brightness').value),
            'contrast': float(self.get_parameter('piece_contrast').value),
            'use_clahe': bool(self.get_parameter('piece_use_clahe').value),
            'clahe_clip_limit': float(self.get_parameter('piece_clahe_clip_limit').value),
            'use_denoise': bool(self.get_parameter('piece_use_denoise').value),
            'use_sharpen': bool(self.get_parameter('piece_use_sharpen').value),
            'saturation_scale': float(self.get_parameter('piece_saturation_scale').value),
            'use_white_balance': bool(self.get_parameter('piece_use_white_balance').value),
        }
        for key, val in overrides.items():
            if val != getattr(defaults, key):
                setattr(cfg, key, val)
        if enabled and preset == 'none' and any(
            overrides[k] != getattr(defaults, k) for k in overrides
        ):
            cfg.enabled = True
        return cfg

    # ------------------------------------------------------------------
    # Camera callback - must be trivially fast, no blocking
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        try:
            # .copy() guarantees we own the buffer (cv_bridge may return a view into DDS memory)
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
            self._last_camera_image = img
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
    # Other callbacks - all fast, no YOLO/ArUco here
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            K = msg.k
            self._camera_matrix = np.array(K, dtype=np.float64).reshape(3, 3)
            D = msg.d
            self._dist_coeffs = np.array(D, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received')

    # ------------------------------------------------------------------
    # Camera→base_link auto-calibration
    # ------------------------------------------------------------------

    def _load_camera_tf(self) -> None:
        """Load camera→base_link 4×4 matrix from camera_config.yaml if saved there."""
        path = str(self.get_parameter('camera_config_file').value)
        if not os.path.isfile(path):
            return
        try:
            with open(path) as f:
                cfg = _yaml.safe_load(f) or {}
            params = cfg.get('static_transform_publisher', {}).get('ros__parameters', {})
            mat = params.get('tf_matrix')
            if mat:
                self._camera_to_base_matrix = np.array(mat, dtype=np.float64).reshape(4, 4)
                self.get_logger().info(
                    f'Camera→base_link TF loaded from {path} — no calibration step needed'
                )
        except Exception as e:
            self.get_logger().warn(f'Could not load camera TF from {path}: {e}')

    def _get_marker_base_positions(self, cal: BoardCalibration) -> dict | None:
        """Compute ArUco marker centre positions in base_link from existing calibration data.

        Uses board_to_base_tf (preferred) or calibration_corners_base as fallback.
        Returns {id: np.array([x,y,z])} for IDs 0-3, or None if no data available.
        """
        if cal.board_to_base_tf is not None:
            tf = np.array(cal.board_to_base_tf, dtype=np.float64)
            return {
                mid: (tf @ np.append(pos, 1.0))[:3]
                for mid, pos in _MARKER_BOARD_M.items()
            }
        if cal.calibration_corners_base is not None and len(cal.calibration_corners_base) == 4:
            a0, i0, _, a9 = [np.array(c, dtype=np.float64) for c in cal.calibration_corners_base]
            X_hat = i0 - a0; X_hat /= np.linalg.norm(X_hat)
            Y_raw = a9 - a0; Y_raw /= np.linalg.norm(Y_raw)
            return {
                mid: a0 + pos[0] * X_hat + pos[1] * Y_raw
                for mid, pos in _MARKER_BOARD_M.items()
            }
        return None

    def _auto_calibrate_camera(self, markers_cam: dict) -> np.ndarray | None:
        """Compute camera→base_link 4×4 TF using Kabsch alignment.

        markers_cam: {id: np.array([x,y,z])} in camera frame (from solvePnP).
        Loads board_calibration.yaml to get known marker positions in base_link.
        Returns 4×4 transform or None on failure.
        """
        try:
            cal_path = resolve_calibration_path(
                self.get_parameter('calibration_file').value, self.get_logger()
            )
            if not os.path.isfile(cal_path):
                self.get_logger().warn(
                    'Auto camera calibration: no board_calibration.yaml found. '
                    'Run calibration_tool once to establish the board frame, '
                    'then re-scan to auto-calibrate the camera.'
                )
                return None
            cal = BoardCalibration.load(cal_path)
            markers_base = self._get_marker_base_positions(cal)
            if markers_base is None:
                self.get_logger().warn(
                    'Auto camera calibration: board_calibration.yaml has no '
                    'board_to_base_tf or calibration_corners_base. '
                    'Run calibration_tool to record board corners first.'
                )
                return None
            ids = sorted(markers_cam.keys())
            pts_cam  = np.array([markers_cam[i]   for i in ids], dtype=np.float64)
            pts_base = np.array([markers_base[i]  for i in ids], dtype=np.float64)
            tf = _kabsch_tf(pts_cam, pts_base)
            residual_mm = float(np.mean(np.linalg.norm(
                pts_base - (tf[:3, :3] @ pts_cam.T + tf[:3, 3:]).T, axis=1
            )) * 1000)
            self.get_logger().info(
                f'Camera auto-calibration done (mean residual: {residual_mm:.1f} mm)'
            )
            return tf
        except Exception as e:
            self.get_logger().error(f'Auto camera calibration exception: {e}')
            return None

    def _save_and_publish_camera_tf(self, tf_matrix: np.ndarray) -> None:
        """Persist 4×4 camera→base_link TF to camera_config.yaml and re-publish as static TF."""
        path = str(self.get_parameter('camera_config_file').value)
        try:
            try:
                with open(path) as f:
                    cfg = _yaml.safe_load(f) or {}
            except Exception:
                cfg = {}
            params = cfg.setdefault(
                'static_transform_publisher', {}
            ).setdefault('ros__parameters', {})
            t = tf_matrix[:3, 3]
            params['x'] = float(t[0])
            params['y'] = float(t[1])
            params['z'] = float(t[2])
            params['tf_matrix'] = tf_matrix.tolist()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                _yaml.dump(cfg, f, default_flow_style=False)
            self.get_logger().info(
                f'Camera→base_link TF saved to {path} '
                '(will load automatically on next startup)'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not save camera TF to {path}: {e}')

        # Publish to /tf_static so RViz and other nodes see the camera frame
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'camera_color_optical_frame'
        t = tf_matrix[:3, 3]
        qx, qy, qz, qw = _rotation_to_quaternion(tf_matrix[:3, :3])
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self._static_broadcaster.sendTransform(msg)
        self.get_logger().info('Camera→base_link static TF published to /tf_static')

    def _human_ready_callback(self, _: Empty) -> None:
        self.get_logger().info('human_ready - forcing move detection notify')
        self._turn_detector.trigger_keyboard_fallback()
        if self._turn_detector.state != TurnDetectorState.IDLE:
            msg = Bool()
            msg.data = True
            self._move_detected_pub.publish(msg)

    def _start_watching_callback(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                state = self._latest_board_state
            if state is not None:
                self._turn_detector.start_watching(np.array(state.grid, dtype=np.int8))
                self.get_logger().info('Turn detection: now watching for human move')
            else:
                self.get_logger().warn('start_watching: no board state yet - using empty baseline')
                self._turn_detector.start_watching(np.zeros(90, dtype=np.int8))
        else:
            self._turn_detector.stop_watching()

    def _get_board_state_callback(self, request, response):
        if request.force_rescan and self._last_camera_image is not None:
            board_state, _, _ = self._process_frame(
                self._last_camera_image.copy(), run_turn_detector=False
            )
            if board_state is not None:
                with self._lock:
                    self._latest_board_state = board_state
                response.board_state = board_state
                response.success = True
                response.message = 'OK (force rescan)'
                return response

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

    def _get_board_transform_callback(self, _request, response):
        """Compute board_to_base_tf and marker 3D positions from ArUco markers.

        Uses cv2.solvePnP on the latest camera frame to estimate each marker's
        3D position in camera frame, then transforms to base_link via TF2.
        The static camera→base_link TF must be published (see camera_config.yaml).
        """
        image = self._last_camera_image
        if image is None:
            response.success = False
            response.message = 'No camera image received yet'
            return response

        if self._camera_matrix is None:
            response.success = False
            response.message = 'Camera intrinsics not received yet (no CameraInfo message)'
            return response

        ok, _H, _dbg, corners_raw, ids_raw = self._board_detector.detect_full(image.copy())
        if not ok or corners_raw is None:
            diag = getattr(self._board_detector, '_last_detect_diag', '')
            response.success = False
            response.message = f'ArUco detection failed: {diag}'
            return response

        markers_cam = self._board_detector.estimate_corner_positions_3d(
            corners_raw, ids_raw, self._camera_matrix, self._dist_coeffs
        )
        if markers_cam is None:
            response.success = False
            response.message = '3D pose estimation failed for one or more markers'
            return response

        # ── Resolve camera→base_link transform ────────────────────────
        # Priority:
        #   1. Already loaded (from camera_config.yaml or a previous auto-calibration)
        #   2. Auto-calibrate now using board_calibration.yaml (saves for next time)
        #   3. TF2 lookup (manual camera_config.yaml with x/y/z set)
        if self._camera_to_base_matrix is not None:
            R = self._camera_to_base_matrix[:3, :3]
            t = self._camera_to_base_matrix[:3, 3]
            markers_base = {mid: R @ pt + t for mid, pt in markers_cam.items()}
        else:
            self.get_logger().info(
                'No camera TF yet — attempting one-time auto-calibration from board_calibration.yaml'
            )
            cam_tf = self._auto_calibrate_camera(markers_cam)
            if cam_tf is not None:
                self._camera_to_base_matrix = cam_tf
                self._save_and_publish_camera_tf(cam_tf)
                R = cam_tf[:3, :3]
                t = cam_tf[:3, 3]
                markers_base = {mid: R @ pt + t for mid, pt in markers_cam.items()}
            else:
                # Last resort: TF2 lookup (requires camera_config.yaml with valid x/y/z)
                try:
                    tf = self._tf_buffer.lookup_transform(
                        'base_link', 'camera_color_optical_frame',
                        RclpyTime(), timeout=RclpyDuration(seconds=1.0),
                    )
                    markers_base = {mid: _apply_transform(tf, pt) for mid, pt in markers_cam.items()}
                except Exception as ex:
                    response.success = False
                    response.message = (
                        f'Camera TF not available: {ex}.\n'
                        'Auto-calibration also failed — board_calibration.yaml needs '
                        'board_to_base_tf or calibration_corners_base.\n'
                        'Fix: run calibration_tool once with the board in place, '
                        'then call get_board_transform again.'
                    )
                    return response

        board_tf = _compute_board_tf_from_markers(markers_base)

        response.board_to_base = board_tf.flatten().tolist()
        flat_centres = []
        for mid in (0, 1, 2, 3):
            flat_centres.extend(markers_base[mid].tolist())
        response.marker_centres_base = flat_centres
        response.success = True
        response.message = 'OK'
        self.get_logger().info(
            'GetBoardTransform: board origin at '
            f'({board_tf[0,3]:.3f}, {board_tf[1,3]:.3f}, {board_tf[2,3]:.3f}) m'
        )
        return response

    # ------------------------------------------------------------------
    # Publish tick - runs on executor thread, trivially fast
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
    # Dedicated detection loop - permanent daemon thread, never on executor
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
    # Core frame processing - pure computation, called only from detection thread
    # ------------------------------------------------------------------

    def _process_frame(
        self, image: np.ndarray, run_turn_detector: bool = True
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
        preprocess_cfg = self._piece_preprocess_config()
        yolo_input = apply_piece_preprocess(warped, preprocess_cfg)
        # Flip horizontally so the debug image matches the dashboard orientation (a0 top-right).
        # YOLO runs on the flipped image; file indices are remapped back below.
        yolo_flipped = cv2.flip(yolo_input, 1)

        grid = np.zeros(90, dtype=np.int8)
        cell_conf = np.zeros(90, dtype=np.float32)
        mean_conf = 0.0

        if self._piece_detector is not None:
            detections, annotated = self._piece_detector.detect(yolo_flipped)
            # Remap file: horizontal flip mirrors file 0↔8
            for det in detections:
                det.file = 8 - det.file
            raw_grid, cell_conf = self._piece_detector.detections_to_grid(detections)
            # Apply per-cell temporal smoothing - a cell value only commits
            # after `grid_smooth_frames` consecutive agreeing detections.
            grid = self._grid_stabilizer.update(raw_grid)
            mean_conf = self._piece_detector.mean_confidence(detections)
            if bool(self.get_parameter('debug_show_preprocess').value):
                debug_out = stack_comparison(cv2.flip(warped, 1), yolo_flipped, annotated)
            else:
                debug_out = annotated
        else:
            debug_out = yolo_flipped if preprocess_cfg.enabled else cv2.flip(warped, 1)

        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.grid = grid.tolist()
        msg.detection_confidence = mean_conf
        msg.cell_confidence = cell_conf.tolist()
        msg.fen = grid_to_fen(msg.grid)

        move_detected = False
        if run_turn_detector:
            grid_arr = np.array(msg.grid, dtype=np.int8)
            _, confirmed = self._turn_detector.update(grid_arr)
            move_detected = bool(confirmed)

        return msg, debug_out, move_detected

    # ------------------------------------------------------------------
    # Debug image publishing - called from executor thread (_publish_tick)
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
        rclpy.spin(node)   # SingleThreadedExecutor - all callbacks are fast, no blocking
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
