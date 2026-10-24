#pragma once
#include "map_server/grid.hpp"
#include <limits>
#include <optional>
namespace map_server
{
enum class SourceStatus : uint8_t
{
  VALID,
  STALE,
  MISSING,
  ERROR
};
enum class RadarSource : uint8_t
{
  ALLY,
  ENEMY
};
struct RadarTarget
{
  uint16_t id{0};
  Eigen::Vector2d xy_map{0, 0};
  double observed_at_s{0}, received_at_s{0}, physical_radius_m{0.3};
};
struct RadarTargetFrame
{
  RadarSource source{RadarSource::ALLY};
  double observation_stamp_s{0}, received_stamp_s{0}; // ROS observation, steady reception
  bool observation_valid{true}, stable_ids{true}, time_degraded{false};
  std::vector<RadarTarget> targets;
};
struct RadarOptions
{
  double stale_after{2.5}, confidence_half_life{2.0}, target_max_age{4.0},
      active_confidence_min{0.25};
  double uncertainty_rate{0.1}, uncertainty_max{0.4}, max_physical_radius{2.0};
  std::optional<uint16_t> self_id;
};
class RadarTargetCache
{
public:
  explicit RadarTargetCache(RadarOptions options = {});
  bool update(const RadarTargetFrame &frame, RadarSource expected);
  void clear();
  SourceStatus status(double ros_now, double steady_now) const;
  double age(double ros_now) const;
  std::vector<RadarTarget> active(double ros_now, double steady_now) const;
  bool selfFilterVerified() const
  {
    return seen_ && !error_ && options_.self_id.has_value() && frame_.stable_ids;
  }
  bool timeDegraded() const
  {
    return frame_.time_degraded;
  }

private:
  RadarOptions options_;
  RadarTargetFrame frame_;
  bool seen_{false}, error_{false};
};
} // namespace map_server
