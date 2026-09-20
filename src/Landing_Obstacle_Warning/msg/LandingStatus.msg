# Tri-state result. UNKNOWN must be treated as unsafe by the flight controller.
uint8 UNKNOWN=0
uint8 LANDABLE=1
uint8 NOT_LANDABLE=2

# Reasons are a bit mask and may be combined.
uint32 REASON_NONE=0
uint32 REASON_TOO_FEW_POINTS=1
uint32 REASON_NO_PLANE=2
uint32 REASON_LOW_COVERAGE=4
uint32 REASON_STABILIZING=8
uint32 REASON_SLOPE=16
uint32 REASON_ROUGHNESS=32
uint32 REASON_OBSTACLE=64
uint32 REASON_AIRSPACE_OCCUPIED=64
uint32 REASON_STEP=128
uint32 REASON_LOW_INLIER_RATIO=256
uint32 REASON_IMU_UNAVAILABLE=512
uint32 REASON_CLOUD_TIMEOUT=1024
uint32 REASON_GRID_TOO_LARGE=2048

std_msgs/Header header
uint8 status
uint8 raw_status
bool status_changed
uint32 reason_mask
string reason
uint32 roi_point_count
float32 slope_deg
float32 roughness_rms
float32 roughness_p95
float32 max_obstacle_height
float32 occupied_cell_ratio
float32 max_height_step
float32 coverage_ratio
float32 inlier_ratio
float32 plane_height
float32 processing_time_ms
