#pragma once

#include <limits>
#include <optional>
#include "map_server/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav_executor
{
// Caller holds the grid mutex. Sweep a circular footprint along path segments.
std::optional<double> findPathObstacle(
  const nav_msgs::msg::Path & path, const map_server::Costmap2D & grid,
  double radius, double begin = 0.0,
  double end = std::numeric_limits<double>::infinity());
bool pathIsClear(const nav_msgs::msg::Path & path, const map_server::Costmap2D & grid,
  double radius);
}  // namespace nav_executor
