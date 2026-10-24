# Only current obstacle geometry, emitted when geometry changes. Health is in MapStatus.
std_msgs/Header header
uint64 producer_id
uint64 version
uint64 input_version
nav_msgs/MapMetaData cloud_info
uint8[] cloud_mask
map_server/ObstacleCircle[] circles
