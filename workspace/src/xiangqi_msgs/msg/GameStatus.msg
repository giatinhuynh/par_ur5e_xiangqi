# High-level game state published by the Game Manager.
std_msgs/Header header

# status values: "idle" | "waiting_human" | "detecting_move" | "computing_ai" | "executing_move" | "game_over"
string status

bool is_red_turn
int32 move_count
string current_fen
string engine_type   # "fairystockfish" | "minimax" | "mcts"
string system_state  # Human-readable description of current BT state

# High-level result classification (orthogonal to status FSM)
# "ongoing" | "red_wins" | "black_wins" | "draw" | "unknown"
string game_result

# Free-text reason, e.g. "checkmate", "stalemate", "repetition", "no_legal_moves"
string game_result_reason
