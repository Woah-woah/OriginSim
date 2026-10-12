#pragma once
#include "map_server/costmap_2d.hpp"
#include "map_server/footprint.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace map_server
{
struct InflationOptions
{
  double radius{0.55};
  double inscribed_radius{0.15};
  double cost_scaling_factor{10.0};
  bool inflate_unknown{false};
  bool inflate_around_unknown{false};
};
// Occupancy (-1, 0..100) is interpreted explicitly at this boundary.
Costmap2D occupancyToCostmap(const nav_msgs::msg::OccupancyGrid & map,
  bool track_unknown, int lethal_threshold = 100, bool trinary = true);
void inflate(Costmap2D & map, const InflationOptions & options);
void clearFootprint(Costmap2D & map, double x, double y, double yaw,
  const Footprint & footprint);
}  // namespace map_server
