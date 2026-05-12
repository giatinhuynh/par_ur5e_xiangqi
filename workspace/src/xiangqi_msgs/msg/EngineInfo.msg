# Real-time AI engine analysis info for the dashboard.
std_msgs/Header header
string engine_type        # "fairystockfish" | "minimax"
int32 depth_reached
int32 nodes_searched
float64 thinking_time_sec
int32 evaluation_cp       # Centipawn score, positive = red advantage
string best_move          # Coordinate notation
string ponder_move        # Anticipated response
