#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "topology_global_planner/topology_plan.hpp"

namespace nav_executor
{
// 正常通道跟随中断应交给脱困处理，不作为定位或传感器故障。
class ConnectorTraversalInterrupted : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

enum class TaskState
{
  IDLE,
  WAITING_FOR_MAP,
  PLANNING,
  PATH_READY,
  RECOVERING,
  RECOVERY_PENDING,
  TERMINAL_FAILED,
  SUCCEEDED,
  FAILED
};

enum class ReplanReason
{
  NONE,
  PATH_DEVIATION,
  EXECUTION_STALLED,
  PATH_BLOCKED,
  MPC_PREDICTED_COLLISION
};

struct PlanIdentity
{
  uint64_t goal_id{0};
  uint64_t plan_generation{0};

  bool operator==(const PlanIdentity & other) const
  {
    return goal_id == other.goal_id && plan_generation == other.plan_generation;
  }

  bool operator!=(const PlanIdentity & other) const
  {
    return !(*this == other);
  }
};

// 分段标识用于进度显示，控制器使用整条规划的标识
struct SegmentIdentity : PlanIdentity
{
  size_t segment_index{0};

  bool operator==(const SegmentIdentity & other) const
  {
    return PlanIdentity::operator==(other) && segment_index == other.segment_index;
  }

  bool operator!=(const SegmentIdentity & other) const
  {
    return !(*this == other);
  }
};

struct ReplanEvent
{
  PlanIdentity identity;
  ReplanReason reason{ReplanReason::NONE};
  double path_distance{0.0};
  double progress{0.0};
  double duration{0.0};
  unsigned int blocked_checks{0};
};

struct PlanningRequest
{
  PlanIdentity identity;
  geometry_msgs::msg::PoseStamped goal;
  std::optional<topology_global_planner::Segment> active_connector;
};

struct PlanningResult
{
  PlanIdentity identity;
  topology_global_planner::TopologyPlan plan;
  std::string error;
  nav_msgs::msg::Path original_path;
  // 规划器报告的阻塞位置，避免把后续拓扑分段起点误当成机器人当前位置。
  std::optional<geometry_msgs::msg::PoseStamped> blocked_start;
  // 主线程确认当前位置实际碰撞或被全局起点净空约束阻塞后填写。
  std::optional<geometry_msgs::msg::PoseStamped> recovery_start;
};

// 仅由串行回调线程访问，所有截止时间使用单调时钟
class TaskManager
{
public:
  using Clock = std::chrono::steady_clock;
  explicit TaskManager(
    double goal_tolerance = 0.02, double retry_delay = 1.0, double replan_cooldown = 2.0, int max_planning_failures = 3);
  bool setGoal(const geometry_msgs::msg::PoseStamped & goal);
  bool replan();
  void cancel();
  std::optional<PlanningRequest> nextRequest(bool map_ready, Clock::time_point now);
  bool accept(PlanningResult result, Clock::time_point now);
  bool executionFailed(PlanIdentity identity, const std::string & error, Clock::time_point now);
  bool beginRecovery(PlanIdentity identity, Clock::time_point now, int max_attempts);
  bool finishRecovery(PlanIdentity identity, bool success, const std::string & error,
    Clock::time_point now);
  bool terminalFailure(PlanIdentity identity, const std::string & error);
  bool recoveryConfirmed(PlanIdentity identity) const;
  bool confirmRecoveryProgress(Clock::time_point now, double stable_time, double minimum_progress);
  const nav_msgs::msg::Path & recoveryRoute() const {return recovery_route_;}
  unsigned int recoveryAttempts() const {return recovery_attempts_;}
  bool executionSucceeded(PlanIdentity identity);
  bool observeProgress(
    PlanIdentity identity, const geometry_msgs::msg::PoseStamped & pose, double entry_clearance);
  bool routeInvalid(const ReplanEvent & event, Clock::time_point now);
  bool isPlanConfirmed(PlanIdentity identity) const;

  PlanIdentity identity() const
  {
    return identity_;
  }

  uint64_t generation() const
  {
    return identity_.plan_generation;
  }

  uint64_t replanCount() const
  {
    return replan_count_;
  }

  ReplanReason lastReplanReason() const
  {
    return last_replan_reason_;
  }

  TaskState state() const
  {
    return state_;
  }

  const topology_global_planner::TopologyPlan & plan() const
  {
    return plan_;
  }

  const topology_global_planner::Segment * currentSegment() const;

  SegmentIdentity segmentIdentity() const
  {
    return SegmentIdentity{identity_, segment_index_};
  }

  const std::string & error() const
  {
    return error_;
  }

  static const char * stateName(TaskState state);
  static const char * reasonName(ReplanReason reason);

private:
  void resetProgress();
  double goal_tolerance_;
  Clock::duration retry_delay_;
  Clock::time_point retry_at_{};
  Clock::duration replan_cooldown_;
  Clock::time_point next_route_plan_at_{};
  bool route_replan_pending_{false}; // 记录路线重规划的调度原因
  PlanIdentity identity_;
  uint64_t replan_count_{0};
  unsigned int recovery_attempts_{0};
  unsigned int planning_failures_{0};
  unsigned int max_planning_failures_{3};
  nav_msgs::msg::Path recovery_route_;
  Clock::time_point recovery_resume_at_{}, recovery_progress_at_{};
  double recovery_resume_progress_{0.0}, recovery_last_progress_{0.0};
  ReplanReason last_replan_reason_{ReplanReason::NONE};
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  TaskState state_{TaskState::IDLE};
  topology_global_planner::TopologyPlan plan_;
  size_t segment_index_{0};
  std::vector<double> path_arcs_;
  std::vector<double> segment_ends_;
  double route_progress_{0.0};
  double connector_progress_{-std::numeric_limits<double>::infinity()};
  std::optional<geometry_msgs::msg::Point> previous_position_;
  // 更换目标、取消任务和规划重试时保留已承诺的通道
  std::optional<topology_global_planner::Segment> active_connector_;
  std::string error_;
};
} // 命名空间 nav_executor
