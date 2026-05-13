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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from xiangqi_msgs.msg import BoardState, GameStatus, MoveHistory, EngineInfo
from xiangqi_msgs.srv import SetEngine

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# Shared application state (updated by ROS callbacks, read by Flask)
_state = {
    'board_grid': [0] * 90,
    'fen': '',
    'game_status': 'idle',
    'is_red_turn': True,
    'move_count': 0,
    'engine_type': 'fairystockfish',
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
}
_state_lock = threading.Lock()

# Flask app (initialised in DashboardNode.start_flask)
_flask_app = Flask(
    __name__,
    template_folder=os.path.join(os.path.dirname(__file__), 'templates'),
    static_folder=os.path.join(os.path.dirname(__file__), 'static'),
)
_flask_app.config['SECRET_KEY'] = 'xiangqi_dashboard'
_socketio = SocketIO(_flask_app, cors_allowed_origins='*', async_mode='threading')

# ROS publisher references (set in DashboardNode.__init__)
_ros_publishers = {}


# ------------------------------------------------------------------
# Flask routes
# ------------------------------------------------------------------

@_flask_app.route('/')
def index():
    return render_template('index.html')


@_flask_app.route('/api/state')
def api_state():
    with _state_lock:
        return jsonify(dict(_state))


@_flask_app.route('/api/new_game', methods=['POST'])
def api_new_game():
    pub = _ros_publishers.get('new_game')
    if pub:
        pub.publish(Empty())
    return jsonify({'ok': True})


@_flask_app.route('/api/emergency_stop', methods=['POST'])
def api_estop():
    pub = _ros_publishers.get('estop')
    active = request.json.get('active', True)
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

@_flask_app.route('/api/resync_from_vision', methods=['POST'])
def api_resync_from_vision():
    pub = _ros_publishers.get('resync')
    if pub:
        pub.publish(Empty())
    return jsonify({'ok': True})


@_flask_app.route('/api/set_engine', methods=['POST'])
def api_set_engine():
    data = request.json
    engine_type = data.get('engine_type', 'fairystockfish')
    difficulty = int(data.get('difficulty', 15))
    cli = _ros_publishers.get('set_engine_cli')
    if cli:
        req = SetEngine.Request()
        req.engine_type = engine_type
        req.difficulty = difficulty
        future = cli.call_async(req)
    return jsonify({'ok': True})


@_socketio.on('connect')
def on_connect():
    with _state_lock:
        emit('state_update', dict(_state))


# ------------------------------------------------------------------
# ROS node
# ------------------------------------------------------------------

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 5000)
        self._port = self.get_parameter('port').value

        # Subscriptions
        self.create_subscription(BoardState, '/xiangqi/board_state', self._board_state_cb, 10)
        self.create_subscription(GameStatus, '/xiangqi/game_status', self._game_status_cb, 10)
        self.create_subscription(MoveHistory, '/xiangqi/move_history', self._move_history_cb, 10)
        self.create_subscription(EngineInfo, '/xiangqi/engine_info', self._engine_info_cb, 10)
        self.create_subscription(Bool, '/xiangqi/gripper_active', self._gripper_cb, 10)
        self.create_subscription(String, '/xiangqi/safety_status', self._safety_cb, 10)
        self.create_subscription(String, '/xiangqi/illegal_move_alert', self._alert_cb, 10)

        # Publishers exposed to Flask
        _ros_publishers['new_game'] = self.create_publisher(Empty, '/xiangqi/new_game', 10)
        _ros_publishers['estop'] = self.create_publisher(Bool, '/xiangqi/emergency_stop', 10)
        _ros_publishers['human_ready'] = self.create_publisher(Empty, '/xiangqi/human_ready', 10)
        _ros_publishers['resync'] = self.create_publisher(Empty, '/xiangqi/resync_from_vision', 10)
        _ros_publishers['set_engine_cli'] = self.create_client(SetEngine, 'set_engine')

        # Start Flask in a background thread
        self._flask_thread = threading.Thread(
            target=lambda: _socketio.run(_flask_app, host='0.0.0.0', port=self._port, log_output=False),
            daemon=True,
        )
        self._flask_thread.start()
        self.get_logger().info(f'Dashboard running at http://0.0.0.0:{self._port}')

    # ------------------------------------------------------------------
    # ROS callbacks -- update shared state and push to websocket
    # ------------------------------------------------------------------

    def _board_state_cb(self, msg: BoardState) -> None:
        with _state_lock:
            _state['board_grid'] = list(msg.grid)
            _state['fen'] = msg.fen
            _state['is_red_turn'] = msg.is_red_turn
            _state['detection_confidence'] = float(msg.detection_confidence)
        self._push_state()

    def _game_status_cb(self, msg: GameStatus) -> None:
        with _state_lock:
            _state['game_status'] = msg.status
            _state['is_red_turn'] = msg.is_red_turn
            _state['move_count'] = msg.move_count
            _state['fen'] = msg.current_fen
            _state['engine_type'] = msg.engine_type
            _state['system_state'] = msg.system_state
        self._push_state()

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
            _state['move_history'].append(entry)
            if len(_state['move_history']) > 200:
                _state['move_history'] = _state['move_history'][-200:]
        self._push_state()

    def _engine_info_cb(self, msg: EngineInfo) -> None:
        with _state_lock:
            _state['evaluation_cp'] = msg.evaluation_cp
            _state['depth_reached'] = msg.depth_reached
            _state['thinking_time'] = round(msg.thinking_time_sec, 2)
            _state['best_move'] = msg.best_move
            _state['ponder_move'] = msg.ponder_move
        self._push_state()

    def _gripper_cb(self, msg: Bool) -> None:
        with _state_lock:
            _state['gripper_active'] = msg.data
        self._push_state()

    def _safety_cb(self, msg: String) -> None:
        with _state_lock:
            _state['estop_active'] = (msg.data != 'OK')
        self._push_state()

    def _alert_cb(self, msg: String) -> None:
        with _state_lock:
            _state['last_alert'] = msg.data
        self._push_state()

    def _push_state(self) -> None:
        with _state_lock:
            snapshot = dict(_state)
        _socketio.emit('state_update', snapshot)


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
