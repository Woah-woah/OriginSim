#pragma once

#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_executor/task_manager.hpp"

namespace nav_executor
{
struct RouteMonitorConfig
{
  bool enabled{true};
  double deviation_distance{0.5};
  double deviation_duration{1.0};
  double stall_duration{3.0};
  double minimum_command_speed{0.1};
  double minimum_progress{0.1};
  double maximum_sample_gap{0.25};
};

// Detects route facts only. TaskManager owns validity, dispatch cooldown and goal state.
// Called with fresh, validated execution feedback in the confirmed path's frame.
class RouteMonitor
{
public:
  explicit RouteMonitor(RouteMonitorConfig config = {});
  std::optional<ReplanEvent> observe(
    const nav_msgs::msg::Path & path, PlanIdentity identity,
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & command,
    TaskManager::Clock::time_point now);
  void reset();

private:
  RouteMonitorConfig config_;
  std::optional<PlanIdentity> identity_;
  std::optional<TaskManager::Clock::time_point> previous_sample_;
  std::optional<TaskManager::Clock::time_point> deviation_since_;
  std::optional<TaskManager::Clock::time_point> stall_since_;
  double progress_anchor_{0.0};
};
}  // namespace nav_executor
