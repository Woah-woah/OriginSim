#include "nav_executor/task_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav_executor
{
TaskManager::TaskManager(double goal_tolerance, double retry_delay)
: goal_tolerance_(goal_tolerance)
{
  if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.0 ||
    !std::isfinite(retry_delay) || retry_delay <= 0.0)
  {
    throw std::invalid_argument("goal_tolerance >= 0 and retry_delay > 0 required");
  }
  retry_delay_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(retry_delay));
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
  return replan();
}

bool TaskManager::replan()
{
  if (!goal_) {return false;}
  ++generation_;
  state_ = TaskState::WAITING_FOR_MAP;
  retry_at_ = Clock::time_point{};
  path_ = nav_msgs::msg::Path{};
  error_.clear();
  return true;
}

void TaskManager::cancel()
{
  ++generation_;
  goal_.reset();
  path_ = nav_msgs::msg::Path{};
  error_.clear();
  state_ = TaskState::IDLE;
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
  ++generation_;
  state_ = TaskState::PLANNING;
  return PlanningRequest{generation_, *goal_};
}

bool TaskManager::accept(PlanningResult result, Clock::time_point now)
{
  if (!goal_ || result.generation != generation_ || state_ != TaskState::PLANNING) {
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

bool TaskManager::executionFailed(uint64_t generation, const std::string & error,
  Clock::time_point now)
{
  if (state_ != TaskState::PATH_READY || generation != generation_) {return false;}
  ++generation_;
  path_ = nav_msgs::msg::Path{};
  error_ = error;
  retry_at_ = now + retry_delay_;
  state_ = TaskState::FAILED;
  return true;
}

bool TaskManager::executionSucceeded(uint64_t generation)
{
  if (state_ != TaskState::PATH_READY || generation != generation_) {return false;}
  state_ = TaskState::SUCCEEDED;
  error_.clear();
  return true;
}
}  // namespace nav_executor
