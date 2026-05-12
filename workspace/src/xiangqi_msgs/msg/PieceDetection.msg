# A single detected chess piece from the YOLO model.
std_msgs/Header header
string piece_type    # e.g. "red_chariot", "black_cannon", "red_general"
int8 file            # 0-8 (a-i), board column
int8 rank            # 0-9, board row (0 = robot/red side)
float32 pixel_x      # Bounding box centre in image frame
float32 pixel_y
float32 confidence   # YOLO detection confidence 0.0-1.0
