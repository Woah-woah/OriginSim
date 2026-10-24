#pragma once
#include "map_server/derived_maps.hpp"
namespace map_server
{
struct DistanceSample
{
  float distance{0};
  double dx{0}, dy{0};
};
bool sampleDistance(const DerivedMaps &maps, double x, double y, DistanceSample &value);
struct DistancePenalty
{
  double cost{0}, derivative{0};
};
// 保留现有 EDT 软代价；车体硬碰撞检查独立使用真实轮廓。
inline DistancePenalty distancePenalty(double distance, double robot_radius,
                                       double inflation_radius, double scaling)
{
  if (distance >= inflation_radius || inflation_radius <= robot_radius)
  {
    return {};
  }
  if (distance <= robot_radius)
  {
    return {1.0, 0.0};
  }
  const double span = inflation_radius - robot_radius;
  const double taper = (inflation_radius - distance) / span;
  const double decay = std::exp(-scaling * (distance - robot_radius));
  return {decay * taper * taper, -decay * (scaling * taper * taper + 2 * taper / span)};
}
} // namespace map_server
