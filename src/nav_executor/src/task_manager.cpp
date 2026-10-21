#include "nav_executor/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav_executor
{
TaskManager::TaskManager(double goal_tolerance, double retry_delay, double replan_cooldown)
    : goal_tolerance_(goal_tolerance)
{
  if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.0 || !std::isfinite(retry_delay) ||
      retry_delay <= 0.0 || !std::isfinite(replan_cooldown) || replan_cooldown < 0.0)
  {
    throw std::invalid_argument(
      "goal_tolerance >= 0, retry_delay > 0, replan_cooldown >= 0 required");
  }
  retry_delay_ =
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(retry_delay));
  replan_cooldown_ =
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(replan_cooldown));
}

bool TaskManager::setGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (goal_ && goal_->header.frame_id == goal.header.frame_id &&
      std::hypot(goal_->pose.position.x - goal.pose.position.x,
        goal_->pose.position.y - goal.pose.position.y) <= goal_tolerance_ &&
      goal_->pose.orientation == goal.pose.orientation)
  {
    return false;
  }
  goal_ = goal;
  ++identity_.goal_id;
  next_route_plan_at_ = Clock::time_point{};
  replan_count_ = 0;
  last_replan_reason_ = ReplanReason::NONE;
  return replan();
}

bool TaskManager::replan()
{
  if (!goal_) {
    return false;
  }
  ++identity_.plan_generation;
  state_ = TaskState::WAITING_FOR_MAP;
  retry_at_ = Clock::time_point{};
  plan_ = {};
  resetProgress();
  error_.clear();
  route_replan_pending_ = false;
  return true;
}

void TaskManager::cancel()
{
  ++identity_.plan_generation;
  goal_.reset();
  plan_ = {};
  resetProgress();
  error_.clear();
  state_ = TaskState::IDLE;
  retry_at_ = next_route_plan_at_ = Clock::time_point{};
  replan_count_ = 0;
  last_replan_reason_ = ReplanReason::NONE;
  route_replan_pending_ = false;
}

std::optional<PlanningRequest> TaskManager::nextRequest(bool map_ready, Clock::time_point now)
{
  if (!goal_ || state_ == TaskState::PLANNING || state_ == TaskState::PATH_READY ||
      state_ == TaskState::SUCCEEDED || now < retry_at_)
  {
    return std::nullopt;
  }
  if (!map_ready) {
    state_ = TaskState::WAITING_FOR_MAP;
    return std::nullopt;
  }
  // 每次规划使用独立代次，同一目标的重试也不例外
  if (route_replan_pending_) {
    next_route_plan_at_ = now + replan_cooldown_;
    route_replan_pending_ = false;
  }
  ++identity_.plan_generation;
  state_ = TaskState::PLANNING;
  return PlanningRequest{identity_, *goal_, active_connector_};
}

bool TaskManager::accept(PlanningResult result, Clock::time_point now)
{
  if (!goal_ || result.identity != identity_ || state_ != TaskState::PLANNING) {
    return false;
  }
  if (result.error.empty()) {
    const auto & segments = result.plan.segments;
    for (size_t i = 0; i < segments.size(); ++i) {
      const auto & segment = segments[i];
      if (segment.path.poses.empty() || segment.path.header.frame_id.empty() ||
          segment.path.header.frame_id != result.plan.full_path.header.frame_id)
      {
        result.error = "Invalid segment path or frame";
        break;
      }
      if (std::any_of(segment.path.poses.begin(), segment.path.poses.end(), [](const auto & pose) {
            const auto & p = pose.pose.position;
            return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
          }))
      {
        result.error = "Invalid segment position";
        break;
      }
      if (i > 0 && segments[i - 1].path.poses.back().pose != segment.path.poses.front().pose) {
        result.error = "Discontinuous segment boundary";
        break;
      }
      if (segment.type == topology_global_planner::SegmentType::CONNECTOR &&
          (segment.connector_id.empty() || segment.from_region.empty() ||
            segment.to_region.empty() ||
            segment.wait_pose.header.frame_id != result.plan.full_path.header.frame_id ||
            segment.exit_pose.header.frame_id != result.plan.full_path.header.frame_id ||
            !std::isfinite(segment.corridor_half_width) || segment.corridor_half_width <= 0.0 ||
            !std::isfinite(segment.wait_pose.pose.position.x) ||
            !std::isfinite(segment.wait_pose.pose.position.y) ||
            !std::isfinite(segment.exit_pose.pose.position.x) ||
            !std::isfinite(segment.exit_pose.pose.position.y) ||
            std::hypot(segment.exit_pose.pose.position.x - segment.wait_pose.pose.position.x,
              segment.exit_pose.pose.position.y - segment.wait_pose.pose.position.y) <= 1e-6))
      {
        result.error = "Invalid connector metadata or exit";
        break;
      }
      if (segment.type == topology_global_planner::SegmentType::CONNECTOR) {
        // 出口截面允许横向偏移，入口和出口保留原始几何参考
        const auto & a = segment.wait_pose.pose.position;
        const auto & b = segment.exit_pose.pose.position;
        const double length = std::hypot(b.x - a.x, b.y - a.y);
        const auto & last = segment.path.poses.back().pose.position;
        const double exit_error =
          ((last.x - b.x) * (b.x - a.x) + (last.y - b.y) * (b.y - a.y)) / length;
        const double lateral =
          std::abs((last.x - a.x) * (b.y - a.y) - (last.y - a.y) * (b.x - a.x)) / length;
        if (std::abs(exit_error) > 1e-5 || lateral > segment.corridor_half_width) {
          result.error = "Connector does not end within exit section";
        }
        if (!result.error.empty()) {
          break;
        }
      }
    }
    if (active_connector_ &&
        (segments.empty() ||
          segments.front().type != topology_global_planner::SegmentType::CONNECTOR ||
          segments.front().connector_id != active_connector_->connector_id ||
          segments.front().from_region != active_connector_->from_region ||
          segments.front().to_region != active_connector_->to_region))
    {
      result.error = "Plan lost committed connector";
    }
  }
  if (result.error.empty()) {
    std::vector<geometry_msgs::msg::PoseStamped> assembled;
    for (const auto & segment : result.plan.segments) {
      assembled.insert(assembled.end(), segment.path.poses.begin(), segment.path.poses.end());
    }
    if (assembled != result.plan.full_path.poses) {
      result.error = "Full path differs from executable segments";
    }
  }
  if (!result.error.empty() || result.plan.full_path.poses.empty() || result.plan.segments.empty())
  {
    error_ = result.error.empty() ? "TopologyPlanner returned an empty path" : result.error;
    plan_ = {};
    resetProgress();
    retry_at_ = now + retry_delay_;
    state_ = TaskState::FAILED;
  } else {
    plan_ = std::move(result.plan);
    resetProgress();
    double arc = 0.0;
    auto previous = plan_.full_path.poses.front().pose.position;
    for (const auto & waypoint : plan_.full_path.poses) {
      const auto & p = waypoint.pose.position;
      arc += std::hypot(p.x - previous.x, p.y - previous.y);
      path_arcs_.push_back(arc);
      previous = p;
    }
    size_t count = 0;
    for (const auto & segment : plan_.segments) {
      count += segment.path.poses.size();
      segment_ends_.push_back(path_arcs_[count - 1]);
    }
    error_.clear();
    state_ = TaskState::PATH_READY;
  }
  return true;
}

const char * TaskManager::stateName(TaskState state)
{
  switch (state) {
  case TaskState::IDLE:
    return "IDLE";
  case TaskState::WAITING_FOR_MAP:
    return "WAITING_FOR_MAP";
  case TaskState::PLANNING:
    return "PLANNING";
  case TaskState::PATH_READY:
    return "PATH_READY";
  case TaskState::SUCCEEDED:
    return "SUCCEEDED";
  case TaskState::FAILED:
    return "FAILED";
  }
  return "UNKNOWN";
}

bool TaskManager::executionFailed(
  PlanIdentity identity, const std::string & error, Clock::time_point now)
{
  if (!isPlanConfirmed(identity)) {
    return false;
  }
  ++identity_.plan_generation;
  plan_ = {};
  resetProgress();
  error_ = error;
  retry_at_ = now + retry_delay_;
  state_ = TaskState::FAILED;
  return true;
}

bool TaskManager::executionSucceeded(PlanIdentity identity)
{
  if (!isPlanConfirmed(identity)) {
    return false;
  }
  active_connector_.reset();
  segment_index_ = plan_.segments.size();
  state_ = TaskState::SUCCEEDED;
  error_.clear();
  return true;
}

const topology_global_planner::Segment * TaskManager::currentSegment() const
{
  if (state_ != TaskState::PATH_READY || segment_index_ >= plan_.segments.size()) {
    return nullptr;
  }
  return &plan_.segments[segment_index_];
}

bool TaskManager::isPlanConfirmed(PlanIdentity identity) const
{
  return goal_.has_value() && state_ == TaskState::PATH_READY && identity == identity_ &&
         !plan_.full_path.poses.empty();
}

void TaskManager::resetProgress()
{
  segment_index_ = 0;
  path_arcs_.clear();
  segment_ends_.clear();
  route_progress_ = 0.0;
  connector_progress_ = -INFINITY;
  previous_position_.reset();
}

bool TaskManager::observeProgress(
  PlanIdentity identity, const geometry_msgs::msg::PoseStamped & pose, double entry_clearance)
{
  if (!isPlanConfirmed(identity)) {
    return false;
  }
  const auto & position = pose.pose.position;
  if (pose.header.frame_id != plan_.full_path.header.frame_id || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(entry_clearance) || entry_clearance <= 0.0)
  {
    throw std::invalid_argument("Invalid route progress pose or entry clearance");
  }
  const size_t previous_index = segment_index_;
  const double travel = previous_position_ ? std::hypot(position.x - previous_position_->x,
                                               position.y - previous_position_->y)
                                           : 0.0;
  previous_position_ = position;

  // 仅在已走位置附近投影，避免交叉路线的相同坐标导致进度跳跃
  const double search_begin = std::max(0.0, route_progress_ - 0.1);
  const double search_end = std::min(path_arcs_.back(), route_progress_ + travel + 0.5);
  double distance = INFINITY;
  double projected = route_progress_;
  const auto & points = plan_.full_path.poses;
  for (size_t i = 1; i < points.size(); ++i) {
    const double begin = path_arcs_[i - 1];
    const double end = path_arcs_[i];
    if (end < search_begin || begin > search_end || end - begin <= 1e-9) {
      continue;
    }
    const auto & a = points[i - 1].pose.position;
    const auto & b = points[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = end - begin;
    const double fraction =
      std::clamp(((position.x - a.x) * dx + (position.y - a.y) * dy) / (length * length),
        std::max(0.0, (search_begin - begin) / length),
        std::min(1.0, (search_end - begin) / length));
    const double candidate =
      std::hypot(position.x - a.x - fraction * dx, position.y - a.y - fraction * dy);
    if (candidate < distance) {
      distance = candidate;
      projected = begin + fraction * length;
    }
  }

  route_progress_ = std::max(route_progress_, projected);
  while (segment_index_ < plan_.segments.size()) {
    const auto & segment = plan_.segments[segment_index_];
    if (segment.type == topology_global_planner::SegmentType::CONNECTOR) {
      const auto & wait = segment.wait_pose.pose.position;
      const auto & exit = segment.exit_pose.pose.position;
      const double dx = exit.x - wait.x;
      const double dy = exit.y - wait.y;
      const double length = std::hypot(dx, dy);
      const double along = ((position.x - wait.x) * dx + (position.y - wait.y) * dy) / length;
      const double lateral =
        std::abs((position.x - wait.x) * dy - (position.y - wait.y) * dx) / length;
      const auto & first = segment.path.poses.front().pose.position;
      const double first_progress = ((first.x - wait.x) * dx + (first.y - wait.y) * dy) / length;
      if (along < std::min(-0.05, first_progress - 0.05) || along < connector_progress_ - 0.05 ||
          lateral > segment.corridor_half_width)
      {
        throw std::runtime_error("Robot left directed connector corridor or reversed traversal");
      }
      connector_progress_ = std::max(connector_progress_, along);
      active_connector_ = segment;
      if (along < length - 1e-6) {
        break;
      }
      route_progress_ = std::max(route_progress_, segment_ends_[segment_index_]);
      active_connector_.reset();
    } else if (route_progress_ < segment_ends_[segment_index_] - 1e-6) {
      break;
    }
    if (segment_index_ + 1 == plan_.segments.size()) {
      break;
    }
    ++segment_index_;
    connector_progress_ = -INFINITY;
  }

  // 接近入口时保留通道承诺，更换目标或重规划后仍沿既定方向走到出口
  const auto * segment = currentSegment();
  if (segment && segment->type == topology_global_planner::SegmentType::NORMAL &&
      segment_index_ + 1 < plan_.segments.size())
  {
    const auto & next = plan_.segments[segment_index_ + 1];
    if (next.type == topology_global_planner::SegmentType::CONNECTOR) {
      const auto & wait = next.wait_pose.pose.position;
      const auto & exit = next.exit_pose.pose.position;
      const double dx = exit.x - wait.x;
      const double dy = exit.y - wait.y;
      const double length = std::hypot(dx, dy);
      const double along = ((position.x - wait.x) * dx + (position.y - wait.y) * dy) / length;
      const double lateral =
        std::abs((position.x - wait.x) * dy - (position.y - wait.y) * dx) / length;
      if (along >= -entry_clearance && along <= length && lateral <= next.corridor_half_width) {
        active_connector_ = next;
      }
    }
  }
  return previous_index != segment_index_;
}

bool TaskManager::routeInvalid(const ReplanEvent & event, Clock::time_point now)
{
  if (!isPlanConfirmed(event.identity) || (event.reason != ReplanReason::PATH_DEVIATION &&
                                            event.reason != ReplanReason::EXECUTION_STALLED &&
                                            event.reason != ReplanReason::PATH_BLOCKED))
  {
    return false;
  }
  // 立即撤销路径并停车，冷却时间仅延迟新规划的调度
  replan();
  retry_at_ = std::max(now, next_route_plan_at_);
  route_replan_pending_ = true;
  last_replan_reason_ = event.reason;
  ++replan_count_;
  return true;
}

const char * TaskManager::reasonName(ReplanReason reason)
{
  switch (reason) {
  case ReplanReason::NONE:
    return "NONE";
  case ReplanReason::PATH_DEVIATION:
    return "PATH_DEVIATION";
  case ReplanReason::EXECUTION_STALLED:
    return "EXECUTION_STALLED";
  case ReplanReason::PATH_BLOCKED:
    return "PATH_BLOCKED";
  }
  return "UNKNOWN";
}
} // 命名空间 nav_executor
