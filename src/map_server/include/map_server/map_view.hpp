#pragma once
#include "map_server/derived_maps.hpp"
#include "map_server/distance_field.hpp"
#include "map_server/static_map.hpp"
#include <algorithm>
#include <memory>
namespace map_server
{
struct MapPolicy
{
  bool require_cloud{true}, require_radar{false};
  double timeout_s{1};
};
struct MapStatusData
{
  DynamicHealth health;
  uint64_t producer_id{0}, derived_version{0}, dynamic_version{0}, input_version{0};
  double stamp_s{0}, received_steady_s{0};
};
class MapClient;
class MapView
{
public:
  MapView() = default;
  MapView(std::shared_ptr<const StaticMap> statics, std::shared_ptr<const DerivedMaps> derived)
      : static_(std::move(statics)), derived_(std::move(derived))
  {
    if (!ready())
      throw std::invalid_argument("MapView requires matching immutable static and derived maps");
  }
  bool ready() const
  {
    return static_ && derived_ && static_producer_ == derived_producer_ &&
           static_->geometry == derived_->geometry && !error_mask_;
  }
  bool current(double ros_now, double steady_now, const MapPolicy &policy) const
  {
    if (!ready() || !dynamic_ || status_.producer_id != static_producer_ ||
        dynamic_producer_ != static_producer_ || derived_->dynamic_version != dynamic_version_ ||
        dynamic_->input_version != derived_->input_version ||
        status_.derived_version != derived_->version ||
        status_.dynamic_version != dynamic_version_ || status_.health.map_error ||
        ros_now < status_.stamp_s - 0.1 || ros_now - status_.stamp_s > policy.timeout_s ||
        steady_now < status_.received_steady_s ||
        steady_now - status_.received_steady_s > policy.timeout_s ||
        (derived_wait_since_ > 0 && steady_now - derived_wait_since_ > policy.timeout_s) ||
        (dynamic_wait_since_ > 0 && steady_now - dynamic_wait_since_ > policy.timeout_s))
      return false;
    return (!policy.require_cloud || status_.health.cloud == SourceStatus::VALID) &&
           (!policy.require_radar || (status_.health.radar_allies == SourceStatus::VALID &&
                                      status_.health.radar_enemies == SourceStatus::VALID &&
                                      status_.health.self_filter_verified));
  }
  // 一次插值同时返回米制距离和世界坐标梯度，查询不会修改冻结视图。
  bool sampleDistance(double x, double y, DistanceSample &sample) const
  {
    return ready() && map_server::sampleDistance(*derived_, x, y, sample);
  }
  // 与前端和平滑器共用同一净空掩码；EDT 来自未膨胀的融合障碍，不重复计算。
  BinaryGrid clearanceMask(double radius) const
  {
    if (!std::isfinite(radius) || radius < 0)
    {
      throw std::invalid_argument("规划净空无效");
    }
    const auto &g = geometry();
    BinaryGrid mask{g, derived().collision_mask.data};
    for (size_t i = 0; i < g.size(); ++i)
    {
      const auto p = g.center(i);
      const double edge = std::min({p.x() - g.origin_x, p.y() - g.origin_y,
                                    g.origin_x + g.width * g.resolution - p.x(),
                                    g.origin_y + g.height * g.resolution - p.y()});
      if (derived().distance_field.data[i] <= radius || edge < radius)
      {
        mask.data[i] = 1;
      }
    }
    return mask;
  }
  const GridGeometry &geometry() const
  {
    if (!ready())
      throw std::logic_error("MapView is not ready");
    return derived_->geometry;
  }
  const DerivedMaps &derived() const
  {
    if (!ready())
      throw std::logic_error("MapView is not ready");
    return *derived_;
  }
  std::shared_ptr<const StaticMap> staticMap() const
  {
    return static_;
  }
  std::shared_ptr<const DerivedMaps> derivedMaps() const
  {
    return derived_;
  }
  std::shared_ptr<const DynamicObstacleView> dynamicObstacles() const
  {
    return dynamic_;
  }
  const MapStatusData &status() const
  {
    return status_;
  }
  uint64_t version() const
  {
    return derived_ ? derived_->version : 0;
  }
  int64_t dynamicStampNs() const
  {
    return static_cast<int64_t>(dynamic_stamp_s_ * 1e9);
  }
  std::optional<size_t> cell(const Eigen::Vector2d &p) const
  {
    return geometry().index(p);
  }
  bool worldToCell(double wx, double wy, unsigned &x, unsigned &y) const
  {
    if (!ready())
      return false;
    auto i = geometry().index({wx, wy});
    if (!i)
      return false;
    x = *i % geometry().width;
    y = *i / geometry().width;
    return true;
  }
  Eigen::Vector2d center(unsigned x, unsigned y) const
  {
    return geometry().center(index(x, y));
  }
  float cost(unsigned x, unsigned y) const
  {
    return derived().costmap.data.at(index(x, y));
  }
  bool occupied(unsigned x, unsigned y) const
  {
    return derived().fused_obstacles.data.at(index(x, y)) != 0;
  }
  bool collision(unsigned x, unsigned y) const
  {
    return derived().collision_mask.data.at(index(x, y)) != 0;
  }
  bool latestOccupied(unsigned x, unsigned y) const
  {
    const auto i = index(x, y);
    if (static_->occupancy.data.at(i))
      return true;
    if (!dynamic_)
      return occupied(x, y);
    const auto p = geometry().center(i);
    const double half = geometry().resolution * 0.5;
    if (dynamic_->cloud_mask)
    {
      const auto &cloud = *dynamic_->cloud_mask;
      const auto &g = cloud.geometry;
      const auto bound = [](double v, unsigned n)
      { return static_cast<int>(std::clamp(v, 0.0, static_cast<double>(n))); };
      const int x0 = bound(std::floor((p.x() - half - g.origin_x) / g.resolution), g.width),
                y0 = bound(std::floor((p.y() - half - g.origin_y) / g.resolution), g.height);
      const int x1 = bound(std::ceil((p.x() + half - g.origin_x) / g.resolution), g.width),
                y1 = bound(std::ceil((p.y() + half - g.origin_y) / g.resolution), g.height);
      for (int cy = y0; cy < y1; ++cy)
        for (int cx = x0; cx < x1; ++cx)
          if (cloud.data.at(static_cast<size_t>(cy) * g.width + cx))
            return true;
    }
    for (const auto &c : dynamic_->circles)
    {
      const double dx = std::max(0.0, std::abs(c.center.x() - p.x()) - half),
                   dy = std::max(0.0, std::abs(c.center.y() - p.y()) - half);
      if (dx * dx + dy * dy <= c.radius_m * c.radius_m)
        return true;
    }
    return false;
  }
  RegionId regionAt(const Eigen::Vector2d &p) const
  {
    if (!static_)
      return INVALID_REGION;
    auto i = static_->geometry.index(p);
    return i ? static_->region_id.data.at(*i) : INVALID_REGION;
  }
  std::optional<float> heightAt(const Eigen::Vector2d &p) const
  {
    if (!static_ || !static_->height)
      return {};
    auto i = static_->geometry.index(p);
    return i ? std::optional<float>(static_->height->data.at(*i)) : std::nullopt;
  }
  std::optional<float> depthAt(const Eigen::Vector2d &p) const
  {
    if (!static_ || !static_->depth)
      return {};
    auto i = static_->geometry.index(p);
    return i ? std::optional<float>(static_->depth->data.at(*i)) : std::nullopt;
  }

private:
  size_t index(unsigned x, unsigned y) const
  {
    const auto &g = geometry();
    if (x >= g.width || y >= g.height)
      throw std::out_of_range("MapView cell");
    return static_cast<size_t>(y) * g.width + x;
  }
  friend class MapClient;
  std::shared_ptr<const StaticMap> static_;
  std::shared_ptr<const DerivedMaps> derived_;
  std::shared_ptr<const DynamicObstacleView> dynamic_;
  MapStatusData status_;
  uint64_t static_producer_{0}, derived_producer_{0}, dynamic_producer_{0}, dynamic_version_{0};
  double dynamic_stamp_s_{0};
  double derived_wait_since_{0}, dynamic_wait_since_{0};
  uint8_t error_mask_{0};
};
} // namespace map_server
