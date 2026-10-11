#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav_executor
{
enum class TaskState {IDLE, WAITING_FOR_MAP, PLANNING, PATH_READY, SUCCEEDED, FAILED};

struct PlanningRequest
{
  uint64_t generation;
  geometry_msgs::msg::PoseStamped goal;
};

struct PlanningResult
{
  uint64_t generation;
  nav_msgs::msg::Path path;
  std::string error;
};

// Owned exclusively by the ROS callback thread. All deadlines use monotonic time.
class TaskManager
{
public:
  using Clock = std::chrono::steady_clock;
  explicit TaskManager(double goal_tolerance = 0.02, double retry_delay = 1.0);
  bool setGoal(const geometry_msgs::msg::PoseStamped & goal);
  bool replan();
  void cancel();
  std::optional<PlanningRequest> nextRequest(bool map_ready, Clock::time_point now);
  bool accept(PlanningResult result, Clock::time_point now);
  bool executionFailed(uint64_t generation, const std::string & error, Clock::time_point now);
  bool executionSucceeded(uint64_t generation);
  uint64_t generation() const {return generation_;}
  TaskState state() const {return state_;}
  const nav_msgs::msg::Path & path() const {return path_;}
  const std::string & error() const {return error_;}
  static const char * stateName(TaskState state);

private:
  double goal_tolerance_;
  Clock::duration retry_delay_;
  Clock::time_point retry_at_{};
  uint64_t generation_{0};
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  TaskState state_{TaskState::IDLE};
  nav_msgs::msg::Path path_;
  std::string error_;
};
}  // namespace nav_executor
