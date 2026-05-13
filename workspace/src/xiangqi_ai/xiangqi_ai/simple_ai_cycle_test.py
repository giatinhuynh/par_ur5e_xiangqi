#!/usr/bin/env python3
"""
simple_ai_cycle_test.py: smoke-test AI → planner → game_manager handshake.

Assumes:
- xiangqi_ai/ai_engine_node
- xiangqi_ai/game_manager_node
- xiangqi_planner/task_planner_node

are already running (e.g. via xiangqi_sim.launch.py).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from xiangqi_msgs.msg import GameStatus, AiMoveCommand, AiExecutionResult


class SimpleAICycleTest(Node):
    def __init__(self) -> None:
        super().__init__('simple_ai_cycle_test')

        self._status_sub = self.create_subscription(
            GameStatus, '/xiangqi/game_status', self._status_cb, 10
        )
        self._cmd_sub = self.create_subscription(
            AiMoveCommand, '/xiangqi/ai_move_command', self._cmd_cb, 10
        )
        self._result_sub = self.create_subscription(
            AiExecutionResult, '/xiangqi/ai_execution_result', self._result_cb, 10
        )

        self._new_game_pub = self.create_publisher(Empty, '/xiangqi/new_game', 10)

        self._latest_status: GameStatus | None = None

        self._started = False
        self._start_timer = self.create_timer(2.0, self._start_if_ready)

    def _start_if_ready(self) -> None:
        if self._started:
            return
        self.get_logger().info('Publishing /xiangqi/new_game for simple AI cycle test')
        self._new_game_pub.publish(Empty())
        self._started = True

    def _status_cb(self, msg: GameStatus) -> None:
        self._latest_status = msg
        if msg.game_result and msg.game_result != 'ongoing':
            self.get_logger().info(
                f'GameStatus: status={msg.status}, '
                f'game_result={msg.game_result}, '
                f'reason={msg.game_result_reason}'
            )

    def _cmd_cb(self, msg: AiMoveCommand) -> None:
        self.get_logger().info(
            f'AiMoveCommand: id={msg.dispatch_id}, '
            f'move={msg.move}, capture={msg.is_capture}, '
            f'expected_fen={msg.expected_fen}'
        )

    def _result_cb(self, msg: AiExecutionResult) -> None:
        status_map = {
            AiExecutionResult.ROBOT_MOVE_COMPLETE: 'ROBOT_MOVE_COMPLETE',
            AiExecutionResult.BOARD_VERIFY_FAILED: 'BOARD_VERIFY_FAILED',
            AiExecutionResult.AI_MOTION_FAILED: 'AI_MOTION_FAILED',
        }
        status_name = status_map.get(msg.status, f'UNKNOWN({msg.status})')
        self.get_logger().info(
            f'AiExecutionResult: id={msg.dispatch_id}, status={status_name}, '
            f'message={msg.message!r}'
        )

        if self._latest_status:
            self.get_logger().info(
                f'Final status snapshot: '
                f'status={self._latest_status.status}, '
                f'game_result={self._latest_status.game_result}, '
                f'reason={self._latest_status.game_result_reason}'
            )
        self.get_logger().info('Simple AI cycle test complete; shutting down.')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleAICycleTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

