#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include "map_server/costmap_2d.hpp"

namespace map_server
{
struct DistanceSample
{
  float distance{0};
  double dx{0}, dy{0};
};

// 不可变快照：距离为米，未知格保留在融合原始栅格中，禁止查询为自由空间。
struct DistanceField
{
  unsigned int width{0}, height{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  double robot_radius{0.15}, inflation_radius{0.55}, cost_scaling_factor{5.0};
  float max_distance{0};  // 饱和上限；局部图在 halo 范围内精确。
  int64_t stamp_ns{0};
  uint64_t version{0};
  std::chrono::steady_clock::time_point received_at{std::chrono::steady_clock::now()};
  std::vector<float> distances;
  std::vector<uint8_t> obstacles;

  bool matches(const Costmap2D & grid) const;
  void validate() const;
  bool sample(double x, double y, DistanceSample & value) const;
};

// 仅接受尚未膨胀的融合障碍图；没有种子时使用显式有限上限。
DistanceField computeDistanceField(const Costmap2D & obstacles, bool include_unknown = true,
  float max_distance = 0);
DistanceField cropDistanceField(const DistanceField & padded, unsigned int halo,
  unsigned int width, unsigned int height, float max_distance);

struct DistancePenalty
{
  double cost{0}, derivative{0};
};

// 软代价与硬碰撞半径分离，影响范围外为零；末端二次窗保证连续梯度。
inline DistancePenalty distancePenalty(double distance, double robot_radius,
  double inflation_radius, double scaling)
{
  if (distance >= inflation_radius || inflation_radius <= robot_radius) {return {};}
  if (distance <= robot_radius) {return {1.0, 0.0};}
  const double span = inflation_radius - robot_radius;
  const double taper = (inflation_radius - distance) / span;
  const double decay = std::exp(-scaling * (distance - robot_radius));
  return {decay * taper * taper, -decay * (scaling * taper * taper + 2 * taper / span)};
}
}  // namespace map_server
