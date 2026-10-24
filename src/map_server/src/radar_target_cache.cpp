#include "map_server/radar_target_cache.hpp"
#include <algorithm>
#include <set>
namespace map_server
{
RadarTargetCache::RadarTargetCache(RadarOptions o) : options_(o)
{
  for (double v : {o.stale_after, o.confidence_half_life, o.target_max_age, o.active_confidence_min,
                   o.max_physical_radius})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid radar options");
  for (double v : {o.uncertainty_rate, o.uncertainty_max})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("Invalid radar uncertainty");
  if (o.active_confidence_min > 1 || o.target_max_age < o.stale_after)
    throw std::invalid_argument("Invalid radar lifetime");
}
void RadarTargetCache::clear()
{
  frame_ = {};
  seen_ = false;
  error_ = false;
}
bool RadarTargetCache::update(const RadarTargetFrame &f, RadarSource expected)
{
  if (f.source != expected || !std::isfinite(f.observation_stamp_s) ||
      !std::isfinite(f.received_stamp_s) || f.observation_stamp_s < 0 || f.received_stamp_s < 0)
  {
    error_ = true;
    return false;
  }
  if (seen_ && (f.observation_stamp_s <= frame_.observation_stamp_s ||
                f.received_stamp_s < frame_.received_stamp_s))
    return false;
  if (!f.observation_valid)
  {
    error_ = true;
    return false;
  }
  std::set<uint16_t> ids;
  for (const auto &t : f.targets)
  {
    if (!t.xy_map.allFinite() || !std::isfinite(t.physical_radius_m) || t.physical_radius_m < 0 ||
        t.physical_radius_m > options_.max_physical_radius || !std::isfinite(t.observed_at_s) ||
        !std::isfinite(t.received_at_s) || t.observed_at_s < 0 ||
        t.observed_at_s > f.observation_stamp_s || t.received_at_s < 0 ||
        t.received_at_s > f.received_stamp_s || (f.stable_ids && !ids.insert(t.id).second))
    {
      error_ = true;
      return false;
    }
  }
  frame_ = f;
  seen_ = true;
  error_ = false;
  return true;
}
double RadarTargetCache::age(double now) const
{
  return seen_ ? now - frame_.observation_stamp_s : std::numeric_limits<double>::infinity();
}
SourceStatus RadarTargetCache::status(double now, double steady) const
{
  if (error_)
    return SourceStatus::ERROR;
  if (!seen_)
    return SourceStatus::MISSING;
  if (age(now) < 0 || steady < frame_.received_stamp_s)
    return SourceStatus::ERROR;
  return age(now) > options_.stale_after || steady - frame_.received_stamp_s > options_.stale_after
             ? SourceStatus::STALE
             : SourceStatus::VALID;
}
std::vector<RadarTarget> RadarTargetCache::active(double now, double steady) const
{
  std::vector<RadarTarget> result;
  if (!seen_)
    return result;
  for (auto t : frame_.targets)
  {
    if (frame_.source == RadarSource::ALLY && frame_.stable_ids && options_.self_id &&
        t.id == *options_.self_id)
      continue;
    const double age = now - t.observed_at_s, received_age = steady - t.received_at_s;
    if (age < 0 || received_age < 0 || std::max(age, received_age) > options_.target_max_age ||
        std::pow(0.5, age / options_.confidence_half_life) < options_.active_confidence_min)
      continue;
    t.physical_radius_m += std::min(options_.uncertainty_max, options_.uncertainty_rate * age);
    result.push_back(t);
  }
  return result;
}
} // namespace map_server
