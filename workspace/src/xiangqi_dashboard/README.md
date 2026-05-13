# xiangqi_dashboard

**Cross-cutting HMI**: Flask + SocketIO web UI bridged to ROS 2 in a single `dashboard_node`.

## Architecture

- **`dashboard_node`** spins `rclpy` and starts the Flask app in a **background thread** so callbacks can update shared Python state under a lock.
- **Subscriptions** populate UI state: `/xiangqi/board_state`, `/xiangqi/game_status`, `/xiangqi/move_history`, `/xiangqi/engine_info`, `/xiangqi/gripper_active`, `/xiangqi/safety_status`.
- **Publishers** for user actions: `/xiangqi/new_game` (`Empty`), `/xiangqi/emergency_stop` (`Bool`), `/xiangqi/human_ready` (`Empty`).

## HTTP / Socket.IO

- `/` serves `templates/index.html` with static CSS/JS.
- `/api/state` returns JSON snapshot for polling clients.
- REST-style POST endpoints (e.g. `/api/new_game`, `/api/emergency_stop`) trigger the ROS publishers above.
- Socket.IO pushes live updates to browsers (`async_mode='threading'`).

## Configuration

- Default listen **port 5000**, `0.0.0.0` (parameter `port` in launch).

## `SetEngine`

The node may expose engine switching through the web layer by calling the **`SetEngine`** service on `ai_engine_node` (see `dashboard_node.py` for routes and client setup).

This package does **not** implement game rules or vision—it only reflects and triggers the running stack.
