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
  if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.0 ||
    !std::isfinite(retry_delay) || retry_delay <= 0.0 ||
    !std::isfinite(replan_cooldown) || replan_cooldown < 0.0)
  {
    throw std::invalid_argument("goal_tolerance >= 0, retry_delay > 0, replan_cooldown >= 0 required");
  }
  retry_delay_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(retry_delay));
  replan_cooldown_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(replan_cooldown));
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
  if (!goal_) {return false;}
  ++identity_.plan_generation;
  state_ = TaskState::WAITING_FOR_MAP;
  retry_at_ = Clock::time_point{};
  plan_ = {};
  segment_index_ = 0;
  error_.clear();
  route_replan_pending_ = false;
  return true;
}

void TaskManager::cancel()
{
  ++identity_.plan_generation;
  goal_.reset();
  plan_ = {};
  segment_index_ = 0;
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
    state_ == TaskState::SUCCEEDED ||
    now < retry_at_)
  {
    return std::nullopt;
  }
  if (!map_ready) {
    state_ = TaskState::WAITING_FOR_MAP;
    return std::nullopt;
  }
  // Every attempt gets a unique generation, including retries of the same target.
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
      if (i > 0 && segments[i - 1].path.poses.back().pose != segment.path.poses.front().pose) {
        result.error = "Discontinuous segment boundary";
        break;
      }
      if (segment.type == topology_global_planner::SegmentType::CONNECTOR &&
        (segment.connector_id.empty() || segment.from_region.empty() || segment.to_region.empty() ||
        segment.path.poses.back().pose != segment.exit_pose.pose))
      {
        result.error = "Invalid connector metadata or exit";
        break;
      }
    }
    if (active_connector_ && (segments.empty() ||
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
  if (!result.error.empty() || result.plan.full_path.poses.empty() ||
    result.plan.segments.empty()) {
    error_ = result.error.empty() ? "TopologyPlanner returned an empty path" : result.error;
    plan_ = {};
    segment_index_ = 0;
    retry_at_ = now + retry_delay_;
    state_ = TaskState::FAILED;
  } else {
    plan_ = std::move(result.plan);
    segment_index_ = 0;
    error_.clear();
    state_ = TaskState::PATH_READY;
  }
  return true;
}

const char * TaskManager::stateName(TaskState state)
{
  switch (state) {
    case TaskState::IDLE: return "IDLE";
    case TaskState::WAITING_FOR_MAP: return "WAITING_FOR_MAP";
    case TaskState::PLANNING: return "PLANNING";
    case TaskState::PATH_READY: return "PATH_READY";
    case TaskState::SUCCEEDED: return "SUCCEEDED";
    case TaskState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

bool TaskManager::executionFailed(SegmentIdentity identity, const std::string & error,
  Clock::time_point now)
{
  if (!isSegmentConfirmed(identity)) {return false;}
  ++identity_.plan_generation;
  plan_ = {};
  segment_index_ = 0;
  error_ = error;
  retry_at_ = now + retry_delay_;
  state_ = TaskState::FAILED;
  return true;
}

bool TaskManager::segmentSucceeded(SegmentIdentity identity)
{
  if (!isSegmentConfirmed(identity)) {return false;}
  if (currentSegment()->type == topology_global_planner::SegmentType::CONNECTOR) {
    active_connector_.reset();
  }
  ++segment_index_;
  if (segment_index_ == plan_.segments.size()) {state_ = TaskState::SUCCEEDED;}
  error_.clear();
  return true;
}

const topology_global_planner::Segment * TaskManager::currentSegment() const
{
  if (state_ != TaskState::PATH_READY || segment_index_ >= plan_.segments.size()) {return nullptr;}
  return &plan_.segments[segment_index_];
}

bool TaskManager::beginSegment(SegmentIdentity identity)
{
  if (!isSegmentConfirmed(identity)) {return false;}
  if (currentSegment()->type == topology_global_planner::SegmentType::CONNECTOR) {
    active_connector_ = *currentSegment();
  }
  return true;
}

bool TaskManager::commitConnector(const topology_global_planner::Segment & connector,
  SegmentIdentity owner)
{
  if (!isSegmentConfirmed(owner) || connector.type != topology_global_planner::SegmentType::CONNECTOR) {
    return false;
  }
  if (active_connector_ && (active_connector_->connector_id != connector.connector_id ||
    active_connector_->from_region != connector.from_region || active_connector_->to_region != connector.to_region))
  {return false;}
  active_connector_ = connector;
  return true;
}

bool TaskManager::isSegmentConfirmed(SegmentIdentity identity) const
{
  return goal_.has_value() && currentSegment() && identity == segmentIdentity() &&
         !currentSegment()->path.poses.empty();
}

bool TaskManager::routeInvalid(const ReplanEvent & event, Clock::time_point now)
{
  if (!isSegmentConfirmed(event.identity) ||
    (event.reason != ReplanReason::PATH_DEVIATION &&
    event.reason != ReplanReason::EXECUTION_STALLED && event.reason != ReplanReason::PATH_BLOCKED))
  {
    return false;
  }
  // Revoke immediately, even during cooldown. Delay dispatch, never safety stopping.
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
    case ReplanReason::NONE: return "NONE";
    case ReplanReason::PATH_DEVIATION: return "PATH_DEVIATION";
    case ReplanReason::EXECUTION_STALLED: return "EXECUTION_STALLED";
    case ReplanReason::PATH_BLOCKED: return "PATH_BLOCKED";
  }
  return "UNKNOWN";
}
}  // namespace nav_executor
