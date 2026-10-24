# Published once per producer: immutable shared map geometry and static semantics.
std_msgs/Header header
uint64 producer_id
nav_msgs/MapMetaData info
uint8[] occupancy
uint16[] region_id
float32[] height
float32[] depth

geometry_msgs/Point[] footprint
string footprint_frame
