# One camera-plane detection in original-image pixels.
int32 track_id
int32 class_id
string class_name
float32 confidence

# Four pixel corners (x, y), either clockwise or counter-clockwise.
# For a horizontal detector this is the axis-aligned rectangle's corners.
geometry_msgs/Polygon polygon

# True when the detector produced an oriented box, false for an axis-aligned box.
bool oriented
