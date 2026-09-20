# Local 2.5D map in the gravity-aligned aircraft frame. Arrays are row-major.
std_msgs/Header header
float32 resolution
uint32 width
uint32 height
float32 origin_x
float32 origin_y
float32 no_data_value

# Fitted plane in output frame: ax + by + cz + d = 0.
float32 plane_a
float32 plane_b
float32 plane_c
float32 plane_d

# Estimated ground Z from plane inliers. no_data_value means no support evidence.
float32[] ground_height
# Highest observed Z in the cell, including objects in the descent corridor.
float32[] highest_point
# Height above the fitted ground plane. Zero means no detected intrusion.
float32[] airspace_intrusion
uint32[] point_count
uint32[] ground_point_count
uint8[] airspace_occupied
