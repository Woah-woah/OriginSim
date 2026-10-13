# Current measured obstacles only; no trajectory prediction.
# data uses raw Costmap uint8 values (0 or 254), never occupancy probabilities.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
nav_msgs/MapMetaData info
uint8[] data
float32[] confidence
bool current
string status
