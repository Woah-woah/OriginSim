#pragma once
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace map_server
{
using Footprint = std::vector<geometry_msgs::msg::Point>;
inline std::pair<double, double> calculateMinAndMaxDistances(const Footprint &footprint)
{
  double minimum = std::numeric_limits<double>::infinity(), maximum = 0.0;
  for (size_t i = 0; i < footprint.size(); ++i)
  {
    const auto &a = footprint[i];
    const auto &b = footprint[(i + 1) % footprint.size()];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 0 ? std::clamp(-(a.x * dx + a.y * dy) / length2, 0.0, 1.0) : 0;
    minimum = std::min(minimum, std::hypot(a.x + t * dx, a.y + t * dy));
    maximum = std::max(maximum, std::hypot(a.x, a.y));
  }
  return {minimum, maximum};
}
inline Footprint transformFootprint(double x, double y, double yaw, const Footprint &input)
{
  Footprint output;
  for (const auto &p : input)
  {
    geometry_msgs::msg::Point q;
    q.x = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
    q.y = y + std::sin(yaw) * p.x + std::cos(yaw) * p.y;
    output.push_back(q);
  }
  return output;
}
inline bool pointInPolygon(double x, double y, const Footprint &polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    const auto &a = polygon[i];
    const auto &b = polygon[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
    {
      inside = !inside;
    }
  }
  return inside;
}
} // namespace map_server
