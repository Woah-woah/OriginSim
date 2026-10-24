# Complete immutable float/binary map from one fused input. Not an OccupancyGrid.
std_msgs/Header header
uint64 producer_id
nav_msgs/MapMetaData info
uint64 version
uint64 input_version
uint8[] fused_obstacles
float32[] distance_field
float32[] costmap
uint8[] collision_mask
float64 generated_at_s
float64 grid_margin_m
float64 hard_radius_m
float64 fusion_ms
float64 edt_ms
float64 costmap_ms
float64 cloud_ms

float64 robot_radius_m
float64 inflation_radius_m
float64 cost_scaling_factor
uint64 dynamic_version
