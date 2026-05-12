# Complete state of the Xiangqi board.
# The grid is a flat array of 90 cells (9 files x 10 ranks),
# indexed as grid[rank * 9 + file], rank 0 = robot side (red).
#
# Cell encoding:
#   0  = empty
#   +1..+7 = red pieces  (1=General, 2=Advisor, 3=Elephant, 4=Horse, 5=Chariot, 6=Cannon, 7=Soldier)
#   -1..-7 = black pieces (same mapping, negative)

std_msgs/Header header
int8[90] grid
string fen          # Position in Xiangqi FEN notation
string last_move    # Last move in coordinate notation, e.g. "h0g2" (empty if start)
bool is_red_turn    # True if it is red's turn to move
float32 detection_confidence  # Mean YOLO confidence across detected pieces
