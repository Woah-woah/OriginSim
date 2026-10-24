#pragma once

#include <functional>
#include <string>

#include "map_server/map_view.hpp"
#include "topology_global_planner/topology_plan.hpp"

namespace minimum_jerk_smoother
{
struct Config
{
  bool enabled{true};
  double timeout{0.3};
  double control_spacing{0.4};
  double corridor_range{1.0};
  double max_speed{1.0};
  double max_acceleration{1.0};
  int collision_iterations{6};
  int topology_iterations{3};
};

struct Result
{
  topology_global_planner::TopologyPlan plan;
  bool smoothed{false};
  std::string reason;
};

// 无 Node、Service 和后台线程；调用者提供同一次规划的地图快照和拓扑校验。
class Smoother
{
public:
  using Plan = topology_global_planner::TopologyPlan;
  using Cancel = std::function<bool()>;
  using Validate = std::function<bool(const Plan &)>;
  explicit Smoother(Config config = {});
  Result smooth(
    const Plan & original, const map_server::MapView & map, double body_radius,
    Validate topology_valid, Cancel cancelled = []() { return false; }) const;

private:
  Config config_;
};
}  // namespace minimum_jerk_smoother
