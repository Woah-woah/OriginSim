#include "map_server/ros_adapters.hpp"
namespace map_server
{
RadarTargetFrame adaptRadar(const msg::RadarTargets &in, RadarSource source, double received)
{
  if (in.header.frame_id != "map" || in.targets.size() > 1000)
    throw std::invalid_argument("Radar requires map coordinates and <=1000 targets");
  RadarTargetFrame out;
  out.source = source;
  out.received_stamp_s = received;
  out.observation_stamp_s = rclcpp::Time(in.header.stamp).seconds();
  out.observation_valid = in.observation_valid;
  out.stable_ids = in.stable_ids;
  out.time_degraded = in.time_degraded;
  // Hardware without synchronized sample timestamps must explicitly stamp at reception and set
  // time_degraded.
  if (out.observation_stamp_s <= 0)
    throw std::invalid_argument("Radar needs an explicit ROS-domain timestamp");
  for (const auto &t : in.targets)
    out.targets.push_back({t.id,
                           {t.position.x, t.position.y},
                           rclcpp::Time(t.observed_at).seconds(),
                           received,
                           t.physical_radius_m});
  return out;
}
} // namespace map_server
