"""
dashboard_node: ROS 2 node that bridges ROS topics to a Flask/SocketIO web dashboard.

Subscribes to:
  /xiangqi/board_state    -> live board grid
  /xiangqi/game_status    -> game state, turn, FEN
  /xiangqi/move_history   -> move log
  /xiangqi/engine_info    -> AI analysis
  /xiangqi/gripper_active -> gripper state
  /xiangqi/safety_status  -> e-stop state

Publishes to (from dashboard UI):
  /xiangqi/new_game       -> New game button
  /xiangqi/emergency_stop -> E-stop button
  /xiangqi/human_ready    -> Manual move confirmation button

Serves web UI at http://0.0.0.0:5000
"""

from __future__ import annotations
import threading
import os
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from xiangqi_msgs.msg import BoardState, GameStatus, MoveHistory, EngineInfo
from xiangqi_msgs.srv import SetEngine

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS

PIECE_CODES_FEN = {
    'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,
    'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,
}

STARTING_FEN = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'
DEFAULT_STOCKFISH_SKILL = 20  # UCI Skill Level 1–20


def fen_to_grid(fen: str) -> list:
    """Parse FEN to flat int8[90] grid - always works without vision."""
    if not fen:
        return [0] * 90
    grid = [0] * 90
    try:
        board_part = fen.split()[0]
        # FEN rows go from rank 10 (top/Black) to rank 1 (bottom/Red)
        for fen_rank_idx, rank_str in enumerate(board_part.split('/')):
            # board_rank: 9 = top (Black home), 0 = bottom (Red home)
            board_rank = 9 - fen_rank_idx
            file_idx = 0
            for ch in rank_str:
                if ch.isdigit():
                    file_idx += int(ch)
                else:
                    code = PIECE_CODES_FEN.get(ch, 0)
                    if code:
                        idx = board_rank * 9 + file_idx
                        grid[idx] = code if ch.isupper() else -code
                    file_idx += 1
    except Exception:
        pass
    return grid


# Shared application state (updated by ROS callbacks, read by Flask)
_state = {
    'board_grid': [0] * 90,  # empty until vision or sim populates it
    'fen': STARTING_FEN,
    'game_status': 'idle',
    'is_red_turn': True,
    'move_count': 0,
    'engine_type': '--',
    'red_engine': 'minimax',
    'black_engine': 'minimax',
    'stockfish_difficulty': DEFAULT_STOCKFISH_SKILL,
    'evaluation_cp': 0,
    'depth_reached': 0,
    'thinking_time': 0.0,
    'best_move': '',
    'ponder_move': '',
    'move_history': [],
    'gripper_active': False,
    'estop_active': False,
    'detection_confidence': 0.0,
    'system_state': 'starting',
    'game_result': 'ongoing',
    'game_result_reason': '',
    'simulation_mode': True,
    'game_mode': 'ai_vs_human',   # 'ai_vs_ai' (sim only) | 'ai_vs_human'
    'human_color': 'red',          # which color the human plays in ai_vs_human
    'last_alert': '',
    '_dirty': False,
}
_state_lock = threading.Lock()


def _json_safe_state(state: dict) -> dict:
    """Copy state for Flask/SocketIO (numpy int8 in board_grid is not JSON-serializable)."""
    out = {k: v for k, v in state.items() if k != '_dirty'}
    grid = out.get('board_grid')
    if grid is not None:
        out['board_grid'] = [int(x) for x in grid]
    return out


def _resolve_dashboard_dirs() -> tuple[str, str]:
    """Templates/static next to source, or under share/xiangqi_dashboard after colcon install."""
    pkg_dir = os.path.dirname(__file__)
    template_dir = os.path.join(pkg_dir, 'templates')
    static_dir = os.path.join(pkg_dir, 'static')
    if os.path.isfile(os.path.join(template_dir, 'index.html')):
        return template_dir, static_dir
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('xiangqi_dashboard')
        template_dir = os.path.join(share, 'templates')
        static_dir = os.path.join(share, 'static')
    except Exception:
        pass
    return template_dir, static_dir


_template_dir, _static_dir = _resolve_dashboard_dirs()

# Flask app
_flask_app = Flask(
    __name__,
    template_folder=_template_dir,
    static_folder=_static_dir,
)
_flask_app.config['SECRET_KEY'] = 'xiangqi_dashboard_2025'
CORS(_flask_app)
# threading: safe to emit from ROS callback/timer threads (eventlet breaks under fast AI play)
# allow_upgrades=False: werkzeug dev server can't handle WebSocket protocol upgrades; polling works fine
_socketio = SocketIO(
    _flask_app,
    cors_allowed_origins='*',
    async_mode='threading',
    ping_timeout=60,
    ping_interval=25,
    allow_upgrades=False,
)

# ROS publisher/client references (set in DashboardNode.__init__)
_ros_publishers = {}
_pending_new_game_mode: str | None = None

# Human move queue: dashboard UI pushes here, ROS thread picks it up
_human_move_queue: list[str] = []
_human_move_lock = threading.Lock()


# ------------------------------------------------------------------
# Flask routes
# ------------------------------------------------------------------

@_flask_app.route('/')
def index():
    return render_template('index.html')


@_flask_app.route('/api/state')
def api_state():
    with _state_lock:
        return jsonify(_json_safe_state(_state))


def _cancel_pending_new_game() -> None:
    global _pending_new_game_mode
    _pending_new_game_mode = None


def _apply_idle_board_state() -> None:
    with _state_lock:
        if _state.get('simulation_mode', False):
            _state['board_grid'] = fen_to_grid(STARTING_FEN)
            _state['fen'] = STARTING_FEN
        _state['move_history'] = []
        _state['move_count'] = 0
        _state['game_status'] = 'idle'
        _state['game_result'] = 'ongoing'
        _state['game_result_reason'] = ''
        _state['best_move'] = ''
        _state['ponder_move'] = ''
        _state['evaluation_cp'] = 0
        _state['depth_reached'] = 0
        _state['thinking_time'] = 0.0
        _state['_dirty'] = True


def _queue_new_game(mode: str) -> None:
    """Publish mode first; new_game fires on ROS timer so game_manager applies self_play."""
    global _pending_new_game_mode
    _publish_game_mode(mode)
    _publish_ai_engines()
    _apply_fairy_stockfish_skill()
    _pending_new_game_mode = mode
    with _state_lock:
        # Sim: show logical start position. Hardware: keep live vision grid until next
        # /xiangqi/board_state (pressing Start was resetting the UI to empty start FEN).
        if _state.get('simulation_mode', False):
            _state['board_grid'] = fen_to_grid(STARTING_FEN)
            _state['fen'] = STARTING_FEN
        _state['move_history'] = []
        _state['move_count'] = 0
        _state['game_result'] = 'ongoing'
        _state['game_result_reason'] = ''
        _state['game_status'] = 'idle'
        _state['_dirty'] = True


@_flask_app.route('/api/new_game', methods=['POST'])
def api_new_game():
    with _state_lock:
        status = (_state.get('game_status') or 'idle').lower()
        if status not in ('idle', 'game_over'):
            return jsonify({
                'ok': False,
                'error': 'Game in progress - use Stop or Reset.',
            }), 409
        mode = _state.get('game_mode', 'ai_vs_human')
    _queue_new_game(mode)
    return jsonify({'ok': True})


@_flask_app.route('/api/stop_game', methods=['POST'])
def api_stop_game():
    _cancel_pending_new_game()
    pub = _ros_publishers.get('stop_game')
    if pub:
        pub.publish(Empty())
    _apply_idle_board_state()
    return jsonify({'ok': True})


@_flask_app.route('/api/reset_game', methods=['POST'])
def api_reset_game():
    """Abort and atomically restart (game_manager reset_game, not stop+deferred new_game)."""
    _cancel_pending_new_game()
    pub = _ros_publishers.get('reset_game')
    if pub:
        pub.publish(Empty())
    with _state_lock:
        _state['move_history'] = []
        _state['move_count'] = 0
        _state['game_result'] = 'ongoing'
        _state['game_result_reason'] = ''
        _state['_dirty'] = True
    return jsonify({'ok': True})


@_flask_app.route('/api/emergency_stop', methods=['POST'])
def api_estop():
    pub = _ros_publishers.get('estop')
    active = request.json.get('active', True) if request.json else True
    if pub:
        msg = Bool()
        msg.data = active
        pub.publish(msg)
    return jsonify({'ok': True})


@_flask_app.route('/api/human_ready', methods=['POST'])
def api_human_ready():
    pub = _ros_publishers.get('human_ready')
    if pub:
        pub.publish(Empty())
    return jsonify({'ok': True})


@_flask_app.route('/api/sync_board', methods=['POST'])
def api_sync_board():
    """Ask game_manager to adopt the latest camera grid as authoritative FEN."""
    pub = _ros_publishers.get('resync')
    if pub is None:
        return jsonify({'ok': False, 'error': 'Resync not available'}), 503
    pub.publish(Empty())
    return jsonify({'ok': True})


def _clamp_stockfish_skill(level: int) -> int:
    return min(max(int(level), 1), 20)


def _apply_fairy_stockfish_skill() -> None:
    """Push UCI Skill Level to Fairy-Stockfish when either side uses it."""
    with _state_lock:
        red = _normalize_engine_name(_state.get('red_engine', 'minimax'))
        black = _normalize_engine_name(_state.get('black_engine', 'minimax'))
        skill = _clamp_stockfish_skill(_state.get('stockfish_difficulty', DEFAULT_STOCKFISH_SKILL))
    if red != 'fairystockfish' and black != 'fairystockfish':
        return
    cli = _ros_publishers.get('set_engine_cli')
    if not cli:
        return
    req = SetEngine.Request()
    req.engine_type = 'fairystockfish'
    req.difficulty = skill
    cli.call_async(req)


@_flask_app.route('/api/set_engine', methods=['POST'])
def api_set_engine():
    data = request.json or {}
    engine_type = data.get('engine_type', 'fairystockfish')
    difficulty = _clamp_stockfish_skill(
        data.get('difficulty', data.get('stockfish_difficulty', DEFAULT_STOCKFISH_SKILL))
    )
    cli = _ros_publishers.get('set_engine_cli')
    if cli:
        req = SetEngine.Request()
        req.engine_type = engine_type
        req.difficulty = difficulty
        cli.call_async(req)
    with _state_lock:
        _state['engine_type'] = engine_type
        if engine_type == 'fairystockfish':
            _state['stockfish_difficulty'] = difficulty
        _state['_dirty'] = True
    return jsonify({'ok': True, 'stockfish_difficulty': difficulty})


@_flask_app.route('/api/set_engines', methods=['POST'])
def api_set_engines():
    """Per-side engines (sim setup only): minimax and/or fairystockfish."""
    if not _mode_change_allowed():
        return jsonify({
            'ok': False,
            'error': 'Cannot change engines during a game. Stop or wait for game over.',
        }), 409
    data = request.json or {}
    red = _normalize_engine_name(data.get('red_engine', data.get('red', 'minimax')))
    black = _normalize_engine_name(data.get('black_engine', data.get('black', 'minimax')))
    with _state_lock:
        prev_skill = _state.get('stockfish_difficulty', DEFAULT_STOCKFISH_SKILL)
    if 'stockfish_difficulty' in data or 'difficulty' in data:
        skill = _clamp_stockfish_skill(
            data.get('stockfish_difficulty', data.get('difficulty', prev_skill))
        )
    else:
        skill = _clamp_stockfish_skill(prev_skill)
    with _state_lock:
        _state['red_engine'] = red
        _state['black_engine'] = black
        _state['engine_type'] = f'{red}|{black}'
        _state['stockfish_difficulty'] = skill
        _state['_dirty'] = True
    _publish_ai_engines()
    _apply_fairy_stockfish_skill()
    return jsonify({
        'ok': True,
        'red_engine': red,
        'black_engine': black,
        'stockfish_difficulty': skill,
    })


@_flask_app.route('/api/set_human_color', methods=['POST'])
def api_set_human_color():
    """Set which color the human plays in ai_vs_human mode."""
    data = request.json or {}
    color = (data.get('color') or '').strip().lower()
    if color not in ('red', 'black'):
        return jsonify({'ok': False, 'error': 'color must be red or black'}), 400
    if not _mode_change_allowed():
        return jsonify({
            'ok': False,
            'error': 'Cannot change side during a game. Stop or wait for game over.',
        }), 409
    with _state_lock:
        _state['human_color'] = color
        _state['_dirty'] = True
    pub = _ros_publishers.get('human_color')
    if pub:
        msg = String()
        msg.data = color
        pub.publish(msg)
    return jsonify({'ok': True, 'human_color': color})


def _publish_game_mode(mode: str) -> None:
    pub = _ros_publishers.get('game_mode')
    if pub:
        msg = String()
        msg.data = mode
        pub.publish(msg)


def _parse_uci_square(move: str, idx: int):
    """Parse file+rank at move[idx:] (UCI ranks 1–10). Returns ((file, rank), next_idx) or None."""
    if idx >= len(move):
        return None
    f = move[idx]
    if f < 'a' or f > 'i':
        return None
    if idx + 2 < len(move) and move[idx + 1] == '1' and move[idx + 2] == '0':
        return (f, 10), idx + 3
    if idx + 1 < len(move) and move[idx + 1].isdigit():
        return (f, int(move[idx + 1])), idx + 2
    return None


def _parse_uci_move(move: str):
    """Return ((from_file, from_rank), (to_file, to_rank)) in UCI coordinates."""
    if not move:
        return None
    a = _parse_uci_square(move, 0)
    if not a:
        return None
    (from_sq, next_i) = a
    b = _parse_uci_square(move, next_i)
    if not b:
        return None
    (to_sq, end_i) = b
    if end_i != len(move):
        return None
    return from_sq, to_sq


def _uci_to_grid_index(file_char: str, uci_rank: int) -> int:
    """UCI rank 1–10 (1=Red home) → board grid index (rank 0–9)."""
    file_i = ord(file_char) - ord('a')
    board_rank = uci_rank - 1
    return board_rank * 9 + file_i


def _normalize_engine_name(name: str) -> str:
    n = (name or '').strip().lower()
    if n in ('stockfish', 'fairy', 'fairystockfish', 'fsf'):
        return 'fairystockfish'
    return 'minimax'


def _publish_ai_engines() -> None:
    import json
    pub = _ros_publishers.get('ai_engines')
    if not pub:
        return
    with _state_lock:
        payload = json.dumps({
            'red': _state.get('red_engine', 'minimax'),
            'black': _state.get('black_engine', 'minimax'),
        })
    msg = String()
    msg.data = payload
    pub.publish(msg)


def _mode_change_allowed() -> bool:
    """Mode may only change before a game starts or after it ends."""
    with _state_lock:
        status = (_state.get('game_status') or 'idle').lower()
    return status in ('idle', 'game_over')


@_flask_app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    """Switch game mode (simulation only). Hardware is always human vs AI on the physical board."""
    data = request.json or {}
    mode = data.get('mode', 'ai_vs_human')
    if mode not in ('ai_vs_ai', 'ai_vs_human'):
        return jsonify({'ok': False, 'error': 'Invalid mode'}), 400
    if not _mode_change_allowed():
        return jsonify({
            'ok': False,
            'error': 'Cannot change mode during a game. Finish the game or press New Game after game over.',
        }), 409
    with _state_lock:
        sim = _state.get('simulation_mode', False)
    if not sim:
        if mode == 'ai_vs_ai':
            return jsonify({
                'ok': False,
                'error': 'AI vs AI is only available in simulation. On hardware, play on the physical board.',
            }), 403
        mode = 'ai_vs_human'
    with _state_lock:
        _state['game_mode'] = mode
        _state['_dirty'] = True
    _publish_game_mode(mode)
    return jsonify({'ok': True, 'mode': mode})


@_flask_app.route('/api/legal_moves', methods=['POST'])
def api_legal_moves():
    """Legal destination squares for a selected piece (pyffish, AI vs Human)."""
    data = request.json or {}
    from_sq = (data.get('from') or '').strip().lower()
    if not from_sq:
        return jsonify({'ok': False, 'error': 'Missing from square', 'dest_indices': []}), 400

    with _state_lock:
        sim = _state.get('simulation_mode', False)
        mode = _state.get('game_mode', 'ai_vs_human')
        fen = _state.get('fen') or STARTING_FEN
        status = (_state.get('game_status') or '').lower()
        is_red = _state.get('is_red_turn', True)
        human_color = _state.get('human_color', 'red')

    if not sim or mode != 'ai_vs_human':
        return jsonify({'ok': False, 'error': 'Only in simulation AI vs Human', 'dest_indices': []}), 403
    human_is_red = human_color == 'red'
    if status != 'waiting_human' or (human_is_red != is_red):
        return jsonify({'ok': False, 'error': 'Not your turn', 'dest_indices': []}), 409

    try:
        import pyffish as sf
        sf.set_option('VariantPath', '')
        legal = sf.legal_moves('xiangqi', fen, [])
    except ImportError:
        return jsonify({'ok': False, 'error': 'pyffish unavailable', 'dest_indices': []}), 503

    dest_indices: list[int] = []
    moves: list[str] = []
    for m in legal:
        parsed = _parse_uci_move(m)
        if not parsed:
            continue
        (ff, fr), (tf, tr) = parsed
        if f'{ff}{fr}' != from_sq:
            continue
        moves.append(m)
        dest_indices.append(_uci_to_grid_index(tf, tr))

    return jsonify({'ok': True, 'from': from_sq, 'dest_indices': dest_indices, 'moves': moves})


@_flask_app.route('/api/simulate_move', methods=['POST'])
def api_simulate_move():
    """Sim only: human click-move on the dashboard board (AI vs Human)."""
    with _state_lock:
        sim = _state.get('simulation_mode', False)
        mode = _state.get('game_mode', 'ai_vs_human')
        fen = _state.get('fen') or STARTING_FEN
    if not sim:
        return jsonify({
            'ok': False,
            'error': 'Moves on the web board are simulation-only. Play on the physical board.',
        }), 403
    if mode != 'ai_vs_human':
        return jsonify({'ok': False, 'error': 'Board clicks only in AI vs Human mode'}), 400
    data = request.json or {}
    move = (data.get('move') or '').strip().lower()
    if not move or _parse_uci_move(move) is None:
        return jsonify({'ok': False, 'error': 'Invalid move format (need e.g. c4c5 or a10a9)'}), 400

    try:
        import pyffish as sf
        sf.set_option('VariantPath', '')
        legal = sf.legal_moves('xiangqi', fen, [])
        if move not in legal:
            return jsonify({'ok': False, 'error': 'Illegal move'}), 400
    except ImportError:
        pass

    with _human_move_lock:
        _human_move_queue.append(move)

    return jsonify({'ok': True, 'move': move})


@_socketio.on('connect')
def on_connect():
    with _state_lock:
        emit('state_update', _json_safe_state(_state))


# ------------------------------------------------------------------
# ROS node
# ------------------------------------------------------------------

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 5000)
        self.declare_parameter('simulation_mode', True)
        self._port = self.get_parameter('port').value
        sim_mode = self.get_parameter('simulation_mode').value
        with _state_lock:
            _state['simulation_mode'] = sim_mode
            _state['game_mode'] = 'ai_vs_ai' if sim_mode else 'ai_vs_human'

        # --- Subscriptions (no duplicates) ---
        self.create_subscription(BoardState, '/xiangqi/board_state', self._board_state_cb, 10)
        self.create_subscription(GameStatus, '/xiangqi/game_status', self._game_status_cb, 10)
        self.create_subscription(MoveHistory, '/xiangqi/move_history', self._move_history_cb, 10)
        self.create_subscription(EngineInfo, '/xiangqi/engine_info', self._engine_info_cb, 10)
        self.create_subscription(Bool, '/xiangqi/gripper_active', self._gripper_cb, 10)
        self.create_subscription(String, '/xiangqi/safety_status', self._safety_cb, 10)
        self.create_subscription(String, '/xiangqi/illegal_move_alert', self._alert_cb, 10)

        # --- Publishers ---
        self._new_game_pub = self.create_publisher(Empty, '/xiangqi/new_game', 10)
        self._human_ready_pub = self.create_publisher(Empty, '/xiangqi/human_ready', 10)
        self._estop_pub = self.create_publisher(Bool, '/xiangqi/emergency_stop', 10)
        self._resync_pub = self.create_publisher(Empty, '/xiangqi/resync_from_vision', 10)
        self._human_move_pub = self.create_publisher(String, '/xiangqi/simulate_human_move', 10)
        self._game_mode_pub = self.create_publisher(String, '/xiangqi/game_mode', 10)
        self._ai_engines_pub = self.create_publisher(String, '/xiangqi/ai_engines', 10)
        self._stop_game_pub = self.create_publisher(Empty, '/xiangqi/stop_game', 10)
        self._reset_game_pub = self.create_publisher(Empty, '/xiangqi/reset_game', 10)
        self._human_color_pub = self.create_publisher(String, '/xiangqi/human_color', 10)

        _ros_publishers['new_game'] = self._new_game_pub
        _ros_publishers['stop_game'] = self._stop_game_pub
        _ros_publishers['reset_game'] = self._reset_game_pub
        _ros_publishers['game_mode'] = self._game_mode_pub
        _ros_publishers['ai_engines'] = self._ai_engines_pub
        _ros_publishers['human_ready'] = self._human_ready_pub
        _ros_publishers['estop'] = self._estop_pub
        _ros_publishers['resync'] = self._resync_pub
        _ros_publishers['human_color'] = self._human_color_pub
        _ros_publishers['set_engine_cli'] = self.create_client(SetEngine, 'set_engine')

        # Push state to browsers (threading async_mode allows emit from this ROS thread)
        self._push_timer = self.create_timer(0.2, self._timer_push_state)
        # Timer: check human move queue every 0.2s
        self._human_move_timer = self.create_timer(0.2, self._check_human_move_queue)
        self._pending_game_timer = self.create_timer(0.12, self._flush_pending_new_game)

        # Start Flask in a background eventlet thread
        self._flask_thread = threading.Thread(
            target=lambda: _socketio.run(
                _flask_app, host='0.0.0.0', port=self._port,
                log_output=False, allow_unsafe_werkzeug=True
            ),
            daemon=True,
        )
        self._flask_thread.start()
        self.get_logger().info(f'Dashboard running at http://0.0.0.0:{self._port}')
        self._mode_sync_timer = self.create_timer(1.0, self._sync_game_mode_once)
        self._mode_synced = False

        # --- Glitch-filter state (hardware mode) ---
        # Board grid hold: only push a new grid to the UI after it has been
        # seen in N consecutive vision messages (or confidence is high).
        self._prev_board_grid: list | None = None
        self._board_grid_repeat: int = 0
        self._BOARD_GRID_HOLD = 2        # consecutive identical msgs before UI update
        self._BOARD_CONF_BYPASS = 0.65   # high-confidence frames skip the hold

        # Detecting-move debounce: suppress the 'detecting_move' label until
        # it has been the reported state for >= N seconds.  This hides the
        # flicker caused by false human-move triggers.
        self._detecting_move_first_seen: float = 0.0
        self._DETECTING_MOVE_DEBOUNCE = 0.4  # seconds

    def _sync_game_mode_once(self) -> None:
        if self._mode_synced:
            return
        self._mode_synced = True
        with _state_lock:
            mode = _state.get('game_mode', 'ai_vs_human')
        _publish_game_mode(mode)
        _publish_ai_engines()
        _apply_fairy_stockfish_skill()
        with _state_lock:
            color = _state.get('human_color', 'red')
        pub = _ros_publishers.get('human_color')
        if pub:
            msg = String()
            msg.data = color
            pub.publish(msg)
        self.get_logger().info(f'Synced game mode to game_manager: {mode} (human={color})')

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------

    def _timer_push_state(self) -> None:
        """Push state snapshot to all connected WebSocket clients."""
        with _state_lock:
            if not _state.get('_dirty', True):
                return
            snapshot = _json_safe_state(_state)
            _state['_dirty'] = False
        try:
            _socketio.emit('state_update', snapshot)
        except Exception as e:
            self.get_logger().warn(f'SocketIO emit failed: {e}')

    def _flush_pending_new_game(self) -> None:
        global _pending_new_game_mode
        if not _pending_new_game_mode:
            return
        mode = _pending_new_game_mode
        _pending_new_game_mode = None
        _publish_game_mode(mode)
        _publish_ai_engines()
        self._new_game_pub.publish(Empty())
        self.get_logger().info(f'Starting game: mode={mode}')

    def _check_human_move_queue(self) -> None:
        """Forward queued human moves to ROS (from AI vs Human mode)."""
        with _human_move_lock:
            moves = list(_human_move_queue)
            _human_move_queue.clear()
        for move in moves:
            msg = String()
            msg.data = move
            self._human_move_pub.publish(msg)
            self.get_logger().info(f'[Dashboard] Human move submitted: {move}')

    # ------------------------------------------------------------------
    # ROS callbacks - update shared state
    # ------------------------------------------------------------------

    def _board_state_cb(self, msg: BoardState) -> None:
        with _state_lock:
            sim = _state.get('simulation_mode', False)
            piece_count = sum(1 for x in msg.grid if x != 0)
            # In sim, vision often publishes an empty grid (no camera/YOLO). Game manager
            # publishes the logical board from FEN after each move.
            if sim and piece_count < 8:
                return
            # On hardware, apply a hold filter: only update the displayed grid
            # when the same grid arrives in N consecutive messages OR confidence
            # is high enough to trust a single frame.
            new_grid = [int(x) for x in msg.grid]
            conf = float(msg.detection_confidence)
            if not sim and conf >= 0.999:
                # Authoritative logical board from game manager (post-move FEN).
                _state['board_grid'] = new_grid
                _state['board_source'] = 'game'
                if msg.fen:
                    _state['fen'] = msg.fen
                _state['detection_confidence'] = conf
                self._prev_board_grid = new_grid
                self._board_grid_repeat = self._BOARD_GRID_HOLD
                _state['_dirty'] = True
                return
            if not sim:
                if new_grid == self._prev_board_grid:
                    self._board_grid_repeat += 1
                else:
                    self._board_grid_repeat = 0
                    self._prev_board_grid = new_grid
                # Suppress the UI update unless the grid is stable or high-confidence
                if self._board_grid_repeat < self._BOARD_GRID_HOLD and conf < self._BOARD_CONF_BYPASS:
                    # Still update non-grid metadata (confidence) but not the grid.
                    # Do not copy is_red_turn from vision - BoardState from camera often
                    # leaves it unset; game_status is authoritative for side to move.
                    _state['detection_confidence'] = conf
                    if msg.fen:
                        _state['fen'] = msg.fen
                    _state['_dirty'] = True
                    return
            _state['board_grid'] = new_grid
            _state['board_source'] = 'vision'
            if msg.fen:
                _state['fen'] = msg.fen
            _state['detection_confidence'] = conf
            _state['_dirty'] = True

    def _game_status_cb(self, msg: GameStatus) -> None:
        with _state_lock:
            new_status = msg.status

            # Debounce 'detecting_move': only show this transient label after
            # it has been the reported state for long enough.  False detections
            # from vision glitches typically flip in and out in < 0.2 s, so
            # they are invisible to the user.
            if new_status == 'detecting_move':
                if self._detecting_move_first_seen == 0.0:
                    self._detecting_move_first_seen = time.time()
                if time.time() - self._detecting_move_first_seen < self._DETECTING_MOVE_DEBOUNCE:
                    # Within debounce window - keep whatever was shown before
                    new_status = _state.get('game_status', new_status)
            else:
                self._detecting_move_first_seen = 0.0

            _state['game_status'] = new_status
            _state['is_red_turn'] = msg.is_red_turn
            prev_moves = _state.get('move_count', 0)
            _state['move_count'] = msg.move_count
            if msg.move_count == 0 and prev_moves > 0:
                _state['move_history'] = []
            _state['engine_type'] = msg.engine_type
            if '|' in (msg.engine_type or ''):
                red, _, black = msg.engine_type.partition('|')
                _state['red_engine'] = _normalize_engine_name(red)
                _state['black_engine'] = _normalize_engine_name(black)
            elif msg.engine_type and msg.status == 'computing_ai':
                side = 'red_engine' if msg.is_red_turn else 'black_engine'
                _state[side] = _normalize_engine_name(msg.engine_type)
            _state['system_state'] = msg.system_state
            _state['game_result'] = getattr(msg, 'game_result', 'ongoing')
            _state['game_result_reason'] = getattr(msg, 'game_result_reason', '')
            # Sim: logical FEN drives the board (no camera). Hardware: vision drives the grid;
            # only update FEN here for game metadata - do not reset to STARTING_FEN on every status tick.
            if msg.current_fen:
                _state['fen'] = msg.current_fen
                if _state.get('simulation_mode', False) or msg.move_count > prev_moves:
                    _state['board_grid'] = fen_to_grid(msg.current_fen)
                    _state['board_source'] = 'fen'
            _state['_dirty'] = True

    def _move_history_cb(self, msg: MoveHistory) -> None:
        entry = {
            'move': msg.move,
            'is_red': msg.is_red_move,
            'time': round(msg.thinking_time_sec, 2),
            'depth': msg.search_depth,
            'eval': msg.evaluation_cp,
            'engine': msg.engine_used,
        }
        with _state_lock:
            if (_state.get('game_status') or '').lower() == 'idle':
                return
            _state['move_history'].append(entry)
            if len(_state['move_history']) > 500:
                _state['move_history'] = _state['move_history'][-500:]
            _state['_dirty'] = True

    def _engine_info_cb(self, msg: EngineInfo) -> None:
        with _state_lock:
            _state['evaluation_cp'] = msg.evaluation_cp
            _state['depth_reached'] = msg.depth_reached
            _state['thinking_time'] = round(msg.thinking_time_sec, 2)
            _state['best_move'] = msg.best_move
            _state['ponder_move'] = msg.ponder_move
            if msg.engine_type:
                _state['engine_type'] = msg.engine_type
            _state['_dirty'] = True

    def _gripper_cb(self, msg: Bool) -> None:
        with _state_lock:
            _state['gripper_active'] = msg.data
            _state['_dirty'] = True

    def _safety_cb(self, msg: String) -> None:
        with _state_lock:
            _state['estop_active'] = (msg.data != 'OK')
            _state['_dirty'] = True

    def _alert_cb(self, msg: String) -> None:
        with _state_lock:
            _state['last_alert'] = msg.data
            _state['_dirty'] = True


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
