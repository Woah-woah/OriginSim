#include "nav_executor/route_monitor.hpp"
#include "nav_executor/path_collision.hpp"

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
    config.maximum_sample_gap, config.blocked_lookahead_distance, config.blocked_maximum_sample_gap})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("RouteMonitor thresholds and durations must be finite and positive");
    }
  }
  if (!std::isfinite(config.blocked_safety_margin) || config.blocked_safety_margin < 0.0 ||
    config.blocked_confirmations < 1)
  {
    throw std::invalid_argument("Invalid path blocked margin or confirmation count");
  }
}

void RouteMonitor::reset()
{
  identity_.reset();
  previous_sample_.reset();
  deviation_since_.reset();
  stall_since_.reset();
  progress_anchor_ = 0.0;
  resetBlocked();
}

void RouteMonitor::resetBlocked()
{
  blocked_identity_.reset();
  blocked_map_stamp_.reset();
  blocked_sample_.reset();
  blocked_since_.reset();
  blocked_count_ = 0;
}

std::optional<ReplanEvent> RouteMonitor::observeBlocked(
  const nav_msgs::msg::Path & path, SegmentIdentity identity,
  const geometry_msgs::msg::PoseStamped & pose, const map_server::Costmap2D & grid,
  int64_t map_stamp, double robot_radius, TaskManager::Clock::time_point now)
{
  const auto & position = pose.pose.position;
  if (!config_.enabled || !config_.path_blocked_enabled || path.poses.empty() ||
    path.header.frame_id.empty() || path.header.frame_id != pose.header.frame_id ||
    !std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(robot_radius) || robot_radius <= 0.0 || map_stamp <= 0 ||
    grid.getSizeInCellsX() == 0 || grid.getSizeInCellsY() == 0)
  {
    resetBlocked();
    return std::nullopt;
  }
  if (!blocked_identity_ || *blocked_identity_ != identity || (blocked_sample_ &&
    (now < *blocked_sample_ || std::chrono::duration<double>(now - *blocked_sample_).count() >
    config_.blocked_maximum_sample_gap)))
  {
    resetBlocked();
    blocked_identity_ = identity;
  }
  if (blocked_map_stamp_ && map_stamp <= *blocked_map_stamp_) {
    if (map_stamp < *blocked_map_stamp_) {resetBlocked();}
    return std::nullopt;
  }
  blocked_map_stamp_ = map_stamp;
  blocked_sample_ = now;

  // Project the robot onto the route, then examine forward arc length, not heading or
  // the straight chord to the goal. Duplicate waypoints and curved paths are valid.
  double best_distance = INFINITY, progress = 0.0, arc = 0.0;
  auto previous = path.poses.front().pose.position;
  for (const auto & waypoint : path.poses) {
    const auto & point = waypoint.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      resetBlocked(); return std::nullopt;
    }
    const double dx = point.x - previous.x, dy = point.y - previous.y;
    const double length = std::hypot(dx, dy);
    const double fraction = length > 1e-9 ? std::clamp(
      ((position.x - previous.x) * dx + (position.y - previous.y) * dy) /
      (length * length), 0.0, 1.0) : 0.0;
    const double distance = std::hypot(position.x - previous.x - fraction * dx,
      position.y - previous.y - fraction * dy);
    if (distance < best_distance) {best_distance = distance; progress = arc + fraction * length;}
    arc += length;
    previous = point;
  }
  const double end = std::min(arc, progress + config_.blocked_lookahead_distance);
  const auto obstacle = findPathObstacle(path, grid,
    robot_radius + config_.blocked_safety_margin, progress, end);
  const double blocked_ahead = obstacle.value_or(-1.0);
  if (blocked_ahead < 0.0) {
    blocked_count_ = 0;
    blocked_since_.reset();
    return std::nullopt;
  }
  if (!blocked_since_) {blocked_since_ = now;}
  ++blocked_count_;
  if (blocked_count_ < static_cast<unsigned int>(config_.blocked_confirmations)) {
    return std::nullopt;
  }
  return ReplanEvent{identity, ReplanReason::PATH_BLOCKED, blocked_ahead, progress,
    std::chrono::duration<double>(now - *blocked_since_).count(), blocked_count_};
}

std::optional<ReplanEvent> RouteMonitor::observe(
  const nav_msgs::msg::Path & path, SegmentIdentity identity,
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
    // Tracking feedback and map snapshots have different cadences. A first tracking
    // sample or an MPPI feedback gap must not erase valid blocked-map evidence.
    previous_sample_.reset();
    deviation_since_.reset();
    stall_since_.reset();
    progress_anchor_ = 0.0;
    if (blocked_identity_ && *blocked_identity_ != identity) {resetBlocked();}
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
