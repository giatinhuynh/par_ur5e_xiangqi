# A single entry in the move history log.
std_msgs/Header header
string move          # Coordinate notation e.g. "h0g2"
bool is_red_move
float64 thinking_time_sec
int32 search_depth
int32 evaluation_cp  # Centipawn evaluation (positive = red advantage)
string engine_used   # "fairystockfish" | "minimax" | "mcts" | "human"
