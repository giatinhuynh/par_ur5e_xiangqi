# High-level game state published by the Game Manager.
std_msgs/Header header

# status values: "idle" | "waiting_human" | "detecting_move" | "computing_ai" | "executing_move" | "red_wins" | "black_wins" | "draw"
string status

bool is_red_turn
int32 move_count
string current_fen
string engine_type   # "fairystockfish" | "minimax"
string system_state  # Human-readable description of current BT state
