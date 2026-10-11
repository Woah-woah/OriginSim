#include "nav_executor/route_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav_executor
{
RouteMonitor::RouteMonitor(RouteMonitorConfig config) : config_(config)
{
  for (const double value : {config.deviation_distance, config.deviation_duration,
    config.stall_duration, config.minimum_command_speed, config.minimum_progress,
    config.maximum_sample_gap})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("RouteMonitor thresholds and durations must be finite and positive");
    }
  }
}

void RouteMonitor::reset()
{
  identity_.reset();
  previous_sample_.reset();
  deviation_since_.reset();
  stall_since_.reset();
  progress_anchor_ = 0.0;
}

std::optional<ReplanEvent> RouteMonitor::observe(
  const nav_msgs::msg::Path & path, PlanIdentity identity,
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & command,
  TaskManager::Clock::time_point now)
{
  const auto & position = pose.pose.position;
  if (!config_.enabled || path.poses.empty() || path.header.frame_id.empty() ||
    pose.header.frame_id != path.header.frame_id || !std::isfinite(position.x) ||
    !std::isfinite(position.y) || !std::isfinite(command.linear.x) ||
    !std::isfinite(command.linear.y))
  {
    reset();
    return std::nullopt;
  }
  if (!identity_ || *identity_ != identity || (previous_sample_ &&
    (now <= *previous_sample_ || std::chrono::duration<double>(now - *previous_sample_).count() >
    config_.maximum_sample_gap)))
  {
    reset();
    identity_ = identity;
  }
  previous_sample_ = now;

  // Project onto segments, not waypoints: sparse paths and duplicate vertices are valid.
  double distance = std::numeric_limits<double>::infinity();
  double progress = 0.0;
  double arc_length = 0.0;
  auto previous = path.poses.front().pose.position;
  for (const auto & waypoint : path.poses) {
    const auto & point = waypoint.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      reset();
      return std::nullopt;
    }
    const double dx = point.x - previous.x;
    const double dy = point.y - previous.y;
    const double length = std::hypot(dx, dy);
    const double fraction = length > 1e-9 ? std::clamp(
      ((position.x - previous.x) * dx + (position.y - previous.y) * dy) /
      (length * length), 0.0, 1.0) : 0.0;
    const double candidate = std::hypot(position.x - previous.x - fraction * dx,
      position.y - previous.y - fraction * dy);
    if (candidate < distance) {
      distance = candidate;
      progress = arc_length + fraction * length;
    }
    arc_length += length;
    previous = point;
  }
  if (distance > config_.deviation_distance) {
    if (!deviation_since_) {deviation_since_ = now;}
    const double duration = std::chrono::duration<double>(now - *deviation_since_).count();
    if (duration >= config_.deviation_duration) {
      return ReplanEvent{identity, ReplanReason::PATH_DEVIATION, distance, progress, duration};
    }
  } else {
    deviation_since_.reset();
  }

  // Pure rotation and deliberate braking do not imply missing translational progress.
  if (std::hypot(command.linear.x, command.linear.y) < config_.minimum_command_speed) {
    stall_since_.reset();
  } else {
    if (!stall_since_ || progress - progress_anchor_ >= config_.minimum_progress) {
      stall_since_ = now;
      progress_anchor_ = progress;
    }
    const double duration = std::chrono::duration<double>(now - *stall_since_).count();
    if (duration >= config_.stall_duration) {
      return ReplanEvent{identity, ReplanReason::EXECUTION_STALLED, distance,
        progress - progress_anchor_, duration};
    }
  }
  return std::nullopt;
}
}  // namespace nav_executor
