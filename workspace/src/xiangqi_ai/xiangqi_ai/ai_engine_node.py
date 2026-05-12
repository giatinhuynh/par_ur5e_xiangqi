"""
ai_engine_node: ROS 2 node providing GetBestMove and SetEngine services.

Acts as a factory: delegates to either FairyStockfishEngine or MinimaxEngine
based on the 'engine_type' ROS parameter (switchable at runtime).
"""

import rclpy
from rclpy.node import Node

from xiangqi_msgs.srv import GetBestMove, SetEngine
from xiangqi_msgs.msg import EngineInfo
from std_msgs.msg import Header

from .fairy_stockfish_engine import FairyStockfishEngine
from .minimax_engine import MinimaxEngine


class AIEngineNode(Node):
    def __init__(self):
        super().__init__('ai_engine_node')

        self.declare_parameter('engine_type', 'fairystockfish')  # or 'minimax'
        self.declare_parameter('difficulty', 15)   # Fairy-Stockfish skill level 1-20
        self.declare_parameter('default_depth', 5)
        self.declare_parameter('default_time_limit', 5.0)

        self._engine_type = self.get_parameter('engine_type').value
        self._difficulty = self.get_parameter('difficulty').value
        self._default_depth = self.get_parameter('default_depth').value
        self._default_time = self.get_parameter('default_time_limit').value

        self._fairy_engine: FairyStockfishEngine | None = None
        self._minimax_engine: MinimaxEngine | None = None
        self._load_engine(self._engine_type)

        self._get_best_move_srv = self.create_service(
            GetBestMove, 'get_best_move', self._get_best_move_cb
        )
        self._set_engine_srv = self.create_service(
            SetEngine, 'set_engine', self._set_engine_cb
        )
        self._engine_info_pub = self.create_publisher(EngineInfo, '/xiangqi/engine_info', 10)

        self.get_logger().info(f'ai_engine_node started with engine: {self._engine_type}')

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _get_best_move_cb(self, request, response):
        engine_type = request.engine_type if request.engine_type else self._engine_type
        depth = request.depth if request.depth > 0 else self._default_depth
        time_limit = request.time_limit if request.time_limit > 0 else self._default_time

        try:
            engine = self._get_engine(engine_type)
            best_move, ponder, depth_reached, eval_cp, elapsed = engine.get_best_move(
                request.fen, depth=depth, time_limit=time_limit
            )

            response.best_move = best_move
            response.ponder_move = ponder
            response.depth_reached = depth_reached
            response.evaluation_cp = eval_cp
            response.thinking_time_sec = elapsed
            response.success = bool(best_move)
            response.message = 'OK' if best_move else 'No move found'

            self._publish_engine_info(engine_type, depth_reached, eval_cp, elapsed, best_move, ponder)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Engine error: {e}')

        return response

    def _set_engine_cb(self, request, response):
        try:
            self._load_engine(request.engine_type)
            self._engine_type = request.engine_type
            if request.engine_type == 'fairystockfish' and self._fairy_engine:
                self._fairy_engine.set_skill_level(request.difficulty)
            response.success = True
            response.message = f'Switched to {request.engine_type}'
            self.get_logger().info(f'Engine switched to: {request.engine_type}')
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ------------------------------------------------------------------
    # Engine management
    # ------------------------------------------------------------------

    def _load_engine(self, engine_type: str) -> None:
        if engine_type == 'fairystockfish':
            if self._fairy_engine is None:
                self.get_logger().info('Starting Fairy-Stockfish engine...')
                self._fairy_engine = FairyStockfishEngine(skill_level=self._difficulty)
                self.get_logger().info('Fairy-Stockfish ready')
        elif engine_type == 'minimax':
            if self._minimax_engine is None:
                self.get_logger().info('Loading minimax engine...')
                self._minimax_engine = MinimaxEngine()
                self.get_logger().info('Minimax engine ready')
        else:
            raise ValueError(f'Unknown engine type: {engine_type}')

    def _get_engine(self, engine_type: str):
        if engine_type == 'fairystockfish':
            if self._fairy_engine is None:
                self._load_engine('fairystockfish')
            return self._fairy_engine
        elif engine_type == 'minimax':
            if self._minimax_engine is None:
                self._load_engine('minimax')
            return self._minimax_engine
        raise ValueError(f'Unknown engine: {engine_type}')

    def _publish_engine_info(self, engine_type, depth, eval_cp, elapsed, best_move, ponder):
        msg = EngineInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.engine_type = engine_type
        msg.depth_reached = depth
        msg.thinking_time_sec = elapsed
        msg.evaluation_cp = eval_cp
        msg.best_move = best_move
        msg.ponder_move = ponder
        self._engine_info_pub.publish(msg)

    def destroy_node(self):
        if self._fairy_engine:
            self._fairy_engine.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AIEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
