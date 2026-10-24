#pragma once
#include "map_server/radar_target_cache.hpp"
#include <memory>
namespace map_server
{
struct DynamicHealth
{
  SourceStatus cloud{SourceStatus::MISSING}, radar_allies{SourceStatus::MISSING},
      radar_enemies{SourceStatus::MISSING};
  double cloud_observation_age_s{INFINITY}, ally_observation_age_s{INFINITY},
      enemy_observation_age_s{INFINITY};
  bool self_filter_verified{false}, radar_time_degraded{false}, map_error{false};
};
struct ObstacleCircle
{
  Eigen::Vector2d center;
  double radius_m;
};
struct DynamicObstacleView
{
  uint64_t input_version{0};
  std::shared_ptr<const BinaryGrid> cloud_mask;
  std::vector<ObstacleCircle> circles;
};
// 动态几何变化与输入心跳分离；静态环境不因收到同样的观测而重算 EDT。
inline bool sameDynamicGeometry(const DynamicObstacleView &a, const DynamicObstacleView &b)
{
  if (bool(a.cloud_mask) != bool(b.cloud_mask) || a.circles.size() != b.circles.size())
  {
    return false;
  }
  if (a.cloud_mask && (!(a.cloud_mask->geometry == b.cloud_mask->geometry) ||
                       a.cloud_mask->data != b.cloud_mask->data))
  {
    return false;
  }
  for (size_t i = 0; i < a.circles.size(); ++i)
  {
    if (a.circles[i].center != b.circles[i].center ||
        a.circles[i].radius_m != b.circles[i].radius_m)
    {
      return false;
    }
  }
  return true;
}
struct DerivedMaps
{
  uint64_t version{0}, input_version{0}; // version = obstacle_version = derived_version
  uint64_t dynamic_version{0};           // ROS 传输中明确绑定同一批动态几何。
  std::shared_ptr<const DynamicObstacleView> dynamic_obstacles;
  double robot_radius_m{0.15}, inflation_radius_m{0.55}, cost_scaling_factor{5.0};
  GridGeometry geometry;
  BinaryGrid fused_obstacles;
  DistanceGrid distance_field;
  CostGrid costmap;
  BinaryGrid collision_mask;
  double generated_at_s{0}, hard_radius_m{0}, grid_margin_m{0};
  double fusion_ms{0}, edt_ms{0}, costmap_ms{0}, cloud_ms{0};
};
} // namespace map_server
