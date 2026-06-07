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
from std_srvs.srv import Trigger
from xiangqi_msgs.srv import SetEngine
from xiangqi_msgs.srv import GetBoardState

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS

PIECE_CODES_FEN = {
    'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,
    'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,
}

STARTING_FEN = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'
DEFAULT_STOCKFISH_SKILL = 20  # UCI Skill Level 1–20


def _grid_to_fen_str(grid: list, template_fen: str) -> str:
    """Rebuild a FEN board-part from a flat int8[90] grid; keep template_fen's metadata tail."""
    PIECE_CHARS = {1: 'K', 2: 'A', 3: 'B', 4: 'N', 5: 'R', 6: 'C', 7: 'P'}
    rows = []
    for rank in range(9, -1, -1):
        row = ''
        empty = 0
        for file in range(9):
            val = grid[rank * 9 + file]
            if val == 0:
                empty += 1
            else:
                if empty:
                    row += str(empty)
                    empty = 0
                ch = PIECE_CHARS.get(abs(val), '?')
                row += ch if val > 0 else ch.lower()
        if empty:
            row += str(empty)
        rows.append(row)
    board_part = '/'.join(rows)
    parts = (template_fen or STARTING_FEN).split()
    parts[0] = board_part
    return ' '.join(parts)


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
    'game_fen': '',  # authoritative FEN from game manager only; empty until first status msg
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
    'game_mode': 'ai_vs_human',   # 'ai_vs_ai' | 'ai_vs_human'
    'human_color': 'red',          # which color the human plays in ai_vs_human
    'last_alert': '',
    # Board scan (hardware pre-game calibration check)
    'board_scan_status': 'none',   # 'none' | 'scanning' | 'ok' | 'fail'
    'board_scan_pieces': 0,
    'prescan_fen': '',             # FEN captured by last successful Scan Board; cleared on new scan
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


def _apply_idle_board_state(preserve_scan: bool = False) -> None:
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
        if not _state.get('simulation_mode', False):
            if preserve_scan:
                # Restore to the pre-game scanned position; keep board_scan_status intact
                prescan = _state.get('prescan_fen', '')
                if prescan:
                    _state['game_fen'] = prescan
                else:
                    _state['board_scan_status'] = 'none'
                    _state['game_fen'] = ''
            else:
                _state['board_scan_status'] = 'none'
                _state['prescan_fen'] = ''
                _state['game_fen'] = ''
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
    _apply_idle_board_state(preserve_scan=True)
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


_SCAN_ROUNDS = 3
_SCAN_INTERVAL = 0.35  # seconds between rounds (matches game_manager startup scan)


def _best_cell_grid(grids: list, cell_confs: list) -> list:
    """Per-cell max-confidence selection across all scan rounds.

    For each of the 90 cells, pick the piece type from whichever round had the
    highest YOLO confidence for that cell. Empty cells (code 0) are always
    overridden by any detection, no matter how low its confidence.
    """
    best_grid = [0] * 90
    best_conf = [0.0] * 90
    for grid, confs in zip(grids, cell_confs):
        for i in range(90):
            if grid[i] != 0 and confs[i] > best_conf[i]:
                best_conf[i] = confs[i]
                best_grid[i] = grid[i]
    return best_grid


def _call_get_board_state(cli, timeout: float = 12.0):
    """Call GetBoardState synchronously from a Flask thread; returns response or None."""
    result_holder = [None]
    done_event = threading.Event()
    req = GetBoardState.Request()
    req.force_rescan = True
    future = cli.call_async(req)

    def _on_done(fut):
        try:
            result_holder[0] = fut.result()
        except Exception:
            pass
        done_event.set()

    future.add_done_callback(_on_done)
    done_event.wait(timeout=timeout)
    return result_holder[0]


@_flask_app.route('/api/scan_board', methods=['POST'])
def api_scan_board():
    """Multi-round scan of the physical board; majority-vote merge for robustness."""
    with _state_lock:
        if _state.get('simulation_mode', False):
            return jsonify({'ok': False, 'error': 'Scan not needed in simulation mode'}), 400
        status = (_state.get('game_status') or 'idle').lower()
        if status not in ('idle', 'game_over'):
            return jsonify({'ok': False, 'error': 'Cannot scan during a game'}), 409
    cli = _ros_publishers.get('get_board_state_cli')
    # Wait up to 5 s for the vision service to become available (common on startup
    # or immediately after a slow YOLO detection frame).
    _SERVICE_WAIT_S = 15.0
    _SERVICE_POLL_S = 0.25
    waited = 0.0
    while (cli is None or not cli.service_is_ready()) and waited < _SERVICE_WAIT_S:
        time.sleep(_SERVICE_POLL_S)
        waited += _SERVICE_POLL_S
        cli = _ros_publishers.get('get_board_state_cli')
    if cli is None or not cli.service_is_ready():
        with _state_lock:
            _state['board_scan_status'] = 'fail'
            _state['_dirty'] = True
        return jsonify({'ok': False, 'error': 'Vision service not ready — check vision node'}), 503

    with _state_lock:
        _state['board_scan_status'] = 'scanning'
        _state['prescan_fen'] = ''
        _state['_dirty'] = True

    grids: list = []
    cell_confs: list = []
    last_fen: str = ''
    for i in range(_SCAN_ROUNDS):
        if i > 0:
            time.sleep(_SCAN_INTERVAL)
        resp = _call_get_board_state(cli)
        if resp is not None and resp.success and resp.board_state is not None:
            grids.append(list(resp.board_state.grid))
            cell_confs.append(list(resp.board_state.cell_confidence) if len(resp.board_state.cell_confidence) == 90 else [0.0] * 90)
            if resp.board_state.fen:
                last_fen = resp.board_state.fen

    if not grids:
        with _state_lock:
            _state['board_scan_status'] = 'fail'
            _state['_dirty'] = True
        return jsonify({'ok': False, 'error': 'Board not detected — check camera and ArUco markers'}), 503

    best_grid = _best_cell_grid(grids, cell_confs)
    piece_count = sum(1 for x in best_grid if x != 0)

    # Rebuild FEN from best frame, using the last scanned FEN for the metadata tail
    scanned_fen = _grid_to_fen_str(best_grid, last_fen or STARTING_FEN)

    with _state_lock:
        _state['board_scan_status'] = 'ok'
        _state['board_scan_pieces'] = piece_count
        _state['game_fen'] = scanned_fen
        _state['prescan_fen'] = scanned_fen
        _state['_dirty'] = True
    return jsonify({'ok': True, 'pieces': piece_count, 'fen': scanned_fen, 'rounds': len(grids)})


@_flask_app.route('/api/move_to_scan_pose', methods=['POST'])
def api_move_to_scan_pose():
    """Command the robot arm to move to the scan (camera) pose."""
    with _state_lock:
        if _state.get('simulation_mode', False):
            return jsonify({'ok': False, 'error': 'Not applicable in simulation mode'}), 400
        status = (_state.get('game_status') or 'idle').lower()
        if status not in ('idle', 'game_over'):
            return jsonify({'ok': False, 'error': 'Cannot move arm during an active game'}), 409
    cli = _ros_publishers.get('move_to_scan_pose_cli')
    if cli is None or not cli.service_is_ready():
        return jsonify({'ok': False, 'error': 'move_to_scan_pose service not ready'}), 503

    result_holder = [None]
    done_event = threading.Event()
    future = cli.call_async(Trigger.Request())

    def _on_done(fut):
        try:
            result_holder[0] = fut.result()
        except Exception:
            pass
        done_event.set()

    future.add_done_callback(_on_done)
    done_event.wait(timeout=120.0)

    resp = result_holder[0]
    if resp is None:
        return jsonify({'ok': False, 'error': 'Timed out waiting for arm (120s)'}), 504
    if not resp.success:
        return jsonify({'ok': False, 'error': resp.message or 'Arm move failed'}), 500
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
    """Switch game mode before Start or after game over (sim and hardware)."""
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
            # Sim: game manager is co-located so STARTING_FEN is valid immediately.
            # Hardware: leave game_fen empty until the game manager sends the real board.
            if sim_mode:
                _state['game_fen'] = STARTING_FEN

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
        self._starting_fen_pub = self.create_publisher(String, '/xiangqi/starting_fen', 10)

        _ros_publishers['new_game'] = self._new_game_pub
        _ros_publishers['stop_game'] = self._stop_game_pub
        _ros_publishers['reset_game'] = self._reset_game_pub
        _ros_publishers['game_mode'] = self._game_mode_pub
        _ros_publishers['ai_engines'] = self._ai_engines_pub
        _ros_publishers['human_ready'] = self._human_ready_pub
        _ros_publishers['estop'] = self._estop_pub
        _ros_publishers['resync'] = self._resync_pub
        _ros_publishers['human_color'] = self._human_color_pub
        _ros_publishers['starting_fen'] = self._starting_fen_pub
        _ros_publishers['set_engine_cli'] = self.create_client(SetEngine, 'set_engine')
        _ros_publishers['get_board_state_cli'] = self.create_client(GetBoardState, 'get_board_state')
        _ros_publishers['move_to_scan_pose_cli'] = self.create_client(
            Trigger, '/xiangqi/move_to_scan_pose'
        )

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
        # Per-cell hysteresis: pieces register immediately when seen; only
        # disappear after N consecutive absent frames.
        # Idle/scanning: high threshold (anti-jitter).
        # Active game: low threshold (moves must reflect quickly).
        self._stable_display_grid: list = [0] * 90
        self._cell_absence_count: list = [0] * 90
        # Per-cell type-change streak: counts consecutive frames where vision
        # reports a different (non-zero) piece than what is currently displayed.
        # A type change only commits after this streak reaches _TYPE_CHANGE_THRESHOLD,
        # preventing brief YOLO misclassifications from flipping the display type.
        self._cell_type_change_streak: list = [0] * 90
        self._cell_type_change_candidate: list = [0] * 90
        self._CELL_ABSENCE_THRESHOLD_IDLE = 3
        self._CELL_ABSENCE_THRESHOLD_GAME = 10  # ~3 s at 3 Hz — piece must be absent consistently
        self._TYPE_CHANGE_THRESHOLD = 6          # ~2 s at 3 Hz — type must be stable before committing
        # Post-move lock: after the game manager publishes an authoritative board
        # (conf=1.0), suppress all vision grid updates for this many seconds so
        # jittery YOLO frames can't overwrite the clean post-move display.
        self._post_move_lock_until: float = 0.0
        self._POST_MOVE_LOCK_SECS = 5.0

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
        # Hardware: if the user pressed Scan Board before Start, forward the
        # resulting FEN so game_manager skips its own 10-round startup rescan.
        with _state_lock:
            is_sim = _state.get('simulation_mode', False)
            scan_ok = _state.get('board_scan_status') == 'ok'
            game_fen = _state.get('game_fen', '')
        if not is_sim and scan_ok and game_fen:
            pub = _ros_publishers.get('starting_fen')
            if pub:
                msg = String()
                msg.data = game_fen
                pub.publish(msg)
                self.get_logger().info(f'Forwarding pre-scanned FEN to game_manager: {game_fen}')
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
            # In sim mode the board is driven exclusively by _game_status_cb (logical FEN).
            # Ignore all /xiangqi/board_state messages to avoid camera data leaking in.
            if _state.get('simulation_mode', False):
                return
            new_grid = [int(x) for x in msg.grid]
            conf = float(msg.detection_confidence)
            if conf >= 0.999:
                # Authoritative logical board from game manager (post-move FEN).
                # Sync hysteresis state so vision resumes from the correct baseline.
                self._stable_display_grid = list(new_grid)
                self._cell_absence_count = [0] * 90
                self._cell_type_change_streak = [0] * 90
                self._cell_type_change_candidate = [0] * 90
                _state['board_grid'] = new_grid
                _state['board_source'] = 'game'
                if msg.fen:
                    _state['fen'] = msg.fen
                _state['detection_confidence'] = conf
                self._post_move_lock_until = time.time() + self._POST_MOVE_LOCK_SECS
                _state['_dirty'] = True
                return
            # Suppress vision grid updates during post-move lock window
            if time.time() < self._post_move_lock_until:
                _state['detection_confidence'] = conf
                _state['_dirty'] = True
                return
            # "What the robot is seeing" is the raw vision view, so each cell tracks
            # what YOLO detects directly:
            #   - a piece registers IMMEDIATELY the first frame it is seen at a cell, and
            #   - each cell locks its piece TYPE to the highest-confidence reading seen
            #     (never downgraded by a lower-confidence frame), so it doesn't jitter.
            # A cell clears once the piece is absent for N consecutive frames (it left),
            # which also resets the per-cell best confidence.
            game_status = _state.get('game_status', 'idle')
            in_game = game_status not in ('idle', 'game_over')
            threshold = (
                self._CELL_ABSENCE_THRESHOLD_IDLE
                if not in_game
                else self._CELL_ABSENCE_THRESHOLD_GAME
            )
            # Per-cell confidence (0.0 when the field is absent/wrong length).
            cell_conf = (
                [float(c) for c in msg.cell_confidence]
                if len(msg.cell_confidence) == 90
                else [0.0] * 90
            )
            for i in range(90):
                vision_val = new_grid[i]
                vision_has = vision_val != 0
                stable_val = self._stable_display_grid[i]
                stable_has = stable_val != 0
                if vision_has:
                    self._cell_absence_count[i] = 0
                    if not stable_has:
                        # Empty → piece: register immediately.
                        self._stable_display_grid[i] = vision_val
                        self._cell_type_change_streak[i] = 0
                        self._cell_type_change_candidate[i] = 0
                    elif vision_val == stable_val:
                        # Same type: streak resets — no pending change.
                        self._cell_type_change_streak[i] = 0
                        self._cell_type_change_candidate[i] = 0
                    else:
                        # Different type (same or different colour): require a
                        # streak before committing so brief YOLO flips don't show.
                        if vision_val == self._cell_type_change_candidate[i]:
                            self._cell_type_change_streak[i] += 1
                        else:
                            self._cell_type_change_candidate[i] = vision_val
                            self._cell_type_change_streak[i] = 1
                        if self._cell_type_change_streak[i] >= self._TYPE_CHANGE_THRESHOLD:
                            self._stable_display_grid[i] = vision_val
                            self._cell_type_change_streak[i] = 0
                            self._cell_type_change_candidate[i] = 0
                else:
                    self._cell_type_change_streak[i] = 0
                    self._cell_type_change_candidate[i] = 0
                    self._cell_absence_count[i] += 1
                    if self._cell_absence_count[i] >= threshold and stable_has:
                        # Piece genuinely left — clear.
                        self._stable_display_grid[i] = 0
            _state['board_grid'] = list(self._stable_display_grid)
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

            prev_status = _state.get('game_status', 'idle')
            _state['game_status'] = new_status
            # When transitioning into an active game, seed the live display from
            # the authoritative FEN so the board never starts from blank.
            if (prev_status in ('idle', 'game_over')
                    and new_status not in ('idle', 'game_over')
                    and not _state.get('simulation_mode', False)):
                seed_fen = _state.get('game_fen') or _state.get('prescan_fen')
                if seed_fen and all(v == 0 for v in self._stable_display_grid):
                    self._stable_display_grid = fen_to_grid(seed_fen)
                    self._cell_absence_count = [0] * 90
                    self._cell_type_change_streak = [0] * 90
                    self._cell_type_change_candidate = [0] * 90
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
                is_sim = _state.get('simulation_mode', False)
                # Hardware: only update game_fen when a game is actually running or has been
                # played (move_count > 0). This prevents the game manager's idle 1-second
                # status tick (which publishes STARTING_FEN) from overwriting the pre-scan
                # empty state or the scan-preview FEN set by /api/scan_board.
                game_active = msg.status not in ('idle', 'game_over') or msg.move_count > 0
                # At game start (no moves yet) keep the scanned position on the FEN board.
                # The game manager is given the prescan FEN, but its early status ticks may
                # briefly publish a different FEN before it applies the scan — don't let
                # that flicker the board. Accept the game FEN only once a move is played.
                prescan = _state.get('prescan_fen', '')
                preserve_scan = (
                    not is_sim and prescan and msg.move_count == 0
                )
                if is_sim or (game_active and not preserve_scan):
                    _state['game_fen'] = msg.current_fen
                if is_sim or msg.move_count > prev_moves:
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
