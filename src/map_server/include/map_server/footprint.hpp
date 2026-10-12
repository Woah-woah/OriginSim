#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "map_server/costmap_2d.hpp"

namespace map_server
{
using Footprint = std::vector<geometry_msgs::msg::Point>;
inline std::pair<double, double> calculateMinAndMaxDistances(const Footprint & footprint)
{
  double minimum = std::numeric_limits<double>::infinity(), maximum = 0.0;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i]; const auto & b = footprint[(i + 1) % footprint.size()];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 0 ? std::clamp(-(a.x * dx + a.y * dy) / length2, 0.0, 1.0) : 0;
    minimum = std::min(minimum, std::hypot(a.x + t * dx, a.y + t * dy));
    maximum = std::max(maximum, std::hypot(a.x, a.y));
  }
  return {minimum, maximum};
}
inline Footprint transformFootprint(double x, double y, double yaw, const Footprint & input)
{
  Footprint output;
  for (const auto & p : input) {
    geometry_msgs::msg::Point q;
    q.x = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
    q.y = y + std::sin(yaw) * p.x + std::cos(yaw) * p.y;
    output.push_back(q);
  }
  return output;
}
inline bool pointInPolygon(double x, double y, const Footprint & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i]; const auto & b = polygon[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
    {inside = !inside;}
  }
  return inside;
}
// Checks polygon edges with Bresenham, preserving Nav2 collision return values.
template<typename GridPointer>
class FootprintCollisionChecker
{
public:
  explicit FootprintCollisionChecker(GridPointer map = nullptr) : map_(map) {}
  void setCostmap(GridPointer map) {map_ = map;}
  GridPointer getCostmap() const {return map_;}
  bool worldToMap(double x, double y, unsigned int & mx, unsigned int & my) const
  {return map_ && map_->worldToMap(x, y, mx, my);}
  double pointCost(int x, int y) const {return map_->getCost(x, y);}
  double footprintCostAtPose(double x, double y, double yaw, const Footprint & polygon) const
  {
    if (!map_ || polygon.size() < 3) {return LETHAL_OBSTACLE;}
    const auto world = transformFootprint(x, y, yaw, polygon);
    double cost = 0.0;
    for (size_t i = 0; i < world.size(); ++i) {
      const auto & a = world[i]; const auto & b = world[(i + 1) % world.size()];
      unsigned int ax, ay, bx, by;
      if (!map_->worldToMap(a.x, a.y, ax, ay) || !map_->worldToMap(b.x, b.y, bx, by))
      {return LETHAL_OBSTACLE;}
      int px = ax, py = ay, tx = bx, ty = by;
      const int dx = std::abs(tx - px), sx = px < tx ? 1 : -1;
      const int dy = -std::abs(ty - py), sy = py < ty ? 1 : -1;
      int error = dx + dy;
      while (true) {
        const auto value = map_->getCost(px, py);
        if (value == LETHAL_OBSTACLE) {return value;}
        cost = std::max(cost, static_cast<double>(value));
        if (px == tx && py == ty) {break;}
        const int twice = 2 * error;
        if (twice >= dy) {error += dy; px += sx;}
        if (twice <= dx) {error += dx; py += sy;}
      }
    }
    return cost;
  }
private:
  GridPointer map_{nullptr};
};
}  // namespace map_server
