#pragma once
#include <string>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace map_server
{
nav_msgs::msg::OccupancyGrid loadStaticMap(const std::string & yaml_filename);
}  // namespace map_server
