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
  path_ = nav_msgs::msg::Path{};
  error_.clear();
  route_replan_pending_ = false;
  return true;
}

void TaskManager::cancel()
{
  ++identity_.plan_generation;
  goal_.reset();
  path_ = nav_msgs::msg::Path{};
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
  return PlanningRequest{identity_, *goal_};
}

bool TaskManager::accept(PlanningResult result, Clock::time_point now)
{
  if (!goal_ || result.identity != identity_ || state_ != TaskState::PLANNING) {
    return false;
  }
  if (!result.error.empty() || result.path.poses.empty()) {
    error_ = result.error.empty() ? "TopologyPlanner returned an empty path" : result.error;
    path_ = nav_msgs::msg::Path{};
    retry_at_ = now + retry_delay_;
    state_ = TaskState::FAILED;
  } else {
    path_ = std::move(result.path);
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

bool TaskManager::executionFailed(PlanIdentity identity, const std::string & error,
  Clock::time_point now)
{
  if (!isPathConfirmed(identity)) {return false;}
  ++identity_.plan_generation;
  path_ = nav_msgs::msg::Path{};
  error_ = error;
  retry_at_ = now + retry_delay_;
  state_ = TaskState::FAILED;
  return true;
}

bool TaskManager::executionSucceeded(PlanIdentity identity)
{
  if (!isPathConfirmed(identity)) {return false;}
  state_ = TaskState::SUCCEEDED;
  error_.clear();
  return true;
}

bool TaskManager::isPathConfirmed(PlanIdentity identity) const
{
  return goal_.has_value() && state_ == TaskState::PATH_READY &&
         identity == identity_ && !path_.poses.empty();
}

bool TaskManager::routeInvalid(const ReplanEvent & event, Clock::time_point now)
{
  if (!isPathConfirmed(event.identity) ||
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
