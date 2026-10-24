# Complete row-major costmap; data is NOT OccupancyGrid probability.
uint8 FREE_SPACE=0
uint8 INSCRIBED_INFLATED_OBSTACLE=253
uint8 LETHAL_OBSTACLE=254
uint8 NO_INFORMATION=255
std_msgs/Header header
nav_msgs/MapMetaData info
uint8[] data
# 同一融合快照的米制 EDT 与未膨胀障碍；局部距离在 halo 范围后饱和。
float32[] edt
uint8[] obstacle_data
float32 edt_max_distance
uint64 obstacle_version
bool current
string robot_base_frame
geometry_msgs/Point[] footprint
float64 robot_radius
bool track_unknown_space
float64 inflation_radius
float64 cost_scaling_factor
