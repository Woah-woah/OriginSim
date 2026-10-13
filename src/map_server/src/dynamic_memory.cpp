#include "map_server/dynamic_memory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include "map_server/cost_values.hpp"

namespace map_server
{
DynamicMemory::DynamicMemory(const DynamicMemoryOptions & options) : options_(options)
{
  for (double value : {options.hit_rate, options.decay_rate, options.on_threshold,
    options.off_threshold, options.obstacle_timeout, options.nominal_dt,
    options.maximum_evidence_dt})
  {if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid dynamic memory setting");}}
  if (options.off_threshold >= options.on_threshold || options.on_threshold > 1.0 ||
    options.maximum_evidence_dt < options.nominal_dt)
  {throw std::invalid_argument("Invalid dynamic confidence thresholds or evidence dt");}
}
void DynamicMemory::clear()
{
  cells_.clear(); initialized_ = false; observed_ = false;
}
bool DynamicMemory::advance(double time, double steady_time)
{
  if (!std::isfinite(time) || !std::isfinite(steady_time))
  {throw std::invalid_argument("Invalid dynamic memory clock");}
  if (initialized_ && (time < time_ || steady_time < steady_time_)) {clear(); return false;}
  const double dt = initialized_ ? std::max(time - time_, steady_time - steady_time_) : 0;
  time_ = time; steady_time_ = steady_time; initialized_ = true;
  for (auto it = cells_.begin(); it != cells_.end();) {
    auto & cell = it->second;
    cell.confidence = std::max(0.0, cell.confidence - options_.decay_rate * dt);
    if (cell.confidence <= options_.off_threshold) {cell.occupied = false;}
    if (std::max(time - cell.ros_seen, steady_time - cell.steady_seen) >=
      options_.obstacle_timeout || cell.confidence == 0)
    {it = cells_.erase(it);} else {++it;}
  }
  return true;
}
void DynamicMemory::observe(const std::vector<DynamicCell> & cells, double time, double steady_time)
{
  if (observed_ && time <= observation_time_)
  {throw std::invalid_argument("Dynamic observations must have increasing timestamps");}
  const double dt = observed_ ? std::min(time - observation_time_, options_.maximum_evidence_dt) :
    options_.nominal_dt;
  std::unordered_set<DynamicCell, DynamicCellHash> unique(cells.begin(), cells.end());
  for (const auto & key : unique) {
    auto entry = cells_.try_emplace(key);
    auto & cell = entry.first->second;
    // A newly appearing cell has only one frame of support, even if the
    // preceding observation gap was long. Do not confirm it with that gap.
    const double evidence_dt = entry.second ? std::min(dt, options_.nominal_dt) : dt;
    cell.confidence = std::min(1.0, cell.confidence + options_.hit_rate * evidence_dt);
    cell.ros_seen = time; cell.steady_seen = steady_time;
    if (cell.confidence >= options_.on_threshold) {cell.occupied = true;}
  }
  observation_time_ = time; observed_ = true;
}
DynamicSnapshot DynamicMemory::snapshot(const DynamicGrid & grid)
{
  DynamicSnapshot result;
  result.data.assign(grid.size(), FREE_SPACE); result.confidence.assign(grid.size(), 0.0f);
  for (auto it = cells_.begin(); it != cells_.end();) {
    if (!grid.contains(it->first)) {it = cells_.erase(it); continue;}
    const auto index = grid.index(it->first);
    result.data[index] = it->second.occupied ? LETHAL_OBSTACLE : FREE_SPACE;
    result.confidence[index] = static_cast<float>(it->second.confidence);
    ++it;
  }
  return result;
}
}  // namespace map_server
