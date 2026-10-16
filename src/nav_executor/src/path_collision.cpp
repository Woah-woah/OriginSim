#include "nav_executor/path_collision.hpp"

#include <algorithm>
#include <cmath>

namespace nav_executor
{
std::optional<double> findPathObstacle(
  const nav_msgs::msg::Path & path, const map_server::Costmap2D & grid,
  double robot_radius, double progress, double end)
{
  if (path.poses.empty() || grid.getSizeInCellsX() == 0 ||
    !std::isfinite(robot_radius) || robot_radius <= 0.0) {return 0.0;}
  // Cell half-diagonal conservatively covers the entire obstacle cell, not just its center.
  const double radius = robot_radius +
    grid.getResolution() * std::sqrt(0.5);
  const double resolution = grid.getResolution();
  double blocked_ahead = -1.0;
  double arc = 0.0;
  auto previous = path.poses.front().pose.position;
  for (const auto & waypoint : path.poses) {
    const auto & point = waypoint.pose.position;
    const double length = std::hypot(point.x - previous.x, point.y - previous.y);
    const double begin_s = std::max(progress, arc), end_s = std::min(end, arc + length);
    if (end_s >= begin_s && arc + length >= progress && arc <= end) {
      const double ux = length > 1e-9 ? (point.x - previous.x) / length : 1.0;
      const double uy = length > 1e-9 ? (point.y - previous.y) / length : 0.0;
      const double ax = previous.x + (begin_s - arc) * ux;
      const double ay = previous.y + (begin_s - arc) * uy;
      const double bx = previous.x + (end_s - arc) * ux;
      const double by = previous.y + (end_s - arc) * uy;
      const auto lower = [](double value, double origin, double resolution) {
          return std::floor((value - origin) / resolution);
        };
      const int x0 = static_cast<int>(std::clamp(lower(std::min(ax, bx) - radius,
          grid.getOriginX(), resolution), 0.0, static_cast<double>(grid.getSizeInCellsX())));
      const int y0 = static_cast<int>(std::clamp(lower(std::min(ay, by) - radius,
          grid.getOriginY(), resolution), 0.0, static_cast<double>(grid.getSizeInCellsY())));
      const int x1 = static_cast<int>(std::clamp(lower(std::max(ax, bx) + radius,
          grid.getOriginX(), resolution), -1.0, static_cast<double>(grid.getSizeInCellsX()) - 1));
      const int y1 = static_cast<int>(std::clamp(lower(std::max(ay, by) + radius,
          grid.getOriginY(), resolution), -1.0, static_cast<double>(grid.getSizeInCellsY()) - 1));
      for (int y = y0; y <= y1 && blocked_ahead < 0.0; ++y) {
        for (int x = x0; x <= x1; ++x) {
          if (grid.getCost(x, y) != map_server::LETHAL_OBSTACLE) {continue;}
          const double wx = grid.getOriginX() + (x + 0.5) * resolution;
          const double wy = grid.getOriginY() + (y + 0.5) * resolution;
          const double along = std::clamp((wx - ax) * ux + (wy - ay) * uy, 0.0, end_s - begin_s);
          if (std::hypot(wx - ax - along * ux, wy - ay - along * uy) <= radius) {
            blocked_ahead = begin_s - progress + along;
            break;
          }
        }
      }
    }
    if (blocked_ahead >= 0.0 || arc >= end) {break;}
    arc += length;
    previous = point;
  }
  if (blocked_ahead < 0.0) {return std::nullopt;}
  return blocked_ahead;
}

bool pathIsClear(const nav_msgs::msg::Path & path, const map_server::Costmap2D & grid,
  double radius)
{
  if (path.poses.empty() || grid.getSizeInCellsX() == 0 ||
    !std::isfinite(radius) || radius <= 0.0) {return false;}
  for (const auto & waypoint : path.poses) {
    const auto & p = waypoint.pose.position;
    unsigned int x, y;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !grid.worldToMap(p.x, p.y, x, y)) {return false;}

  }
  return !findPathObstacle(path, grid, radius);
}
}  // namespace nav_executor
