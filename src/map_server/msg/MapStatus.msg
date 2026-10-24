# Lightweight heartbeat. No map arrays or target lists.
std_msgs/Header header
uint64 producer_id
uint64 derived_version
uint64 dynamic_version
uint64 input_version
map_server/DynamicHealth health
string worker_error
