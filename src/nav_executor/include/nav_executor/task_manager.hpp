#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "topology_global_planner/topology_plan.hpp"

namespace nav_executor
{
enum class TaskState {IDLE, WAITING_FOR_MAP, PLANNING, PATH_READY, SUCCEEDED, FAILED};
enum class ReplanReason {NONE, PATH_DEVIATION, EXECUTION_STALLED, PATH_BLOCKED};

struct PlanIdentity
{
  uint64_t goal_id{0};
  uint64_t plan_generation{0};
  bool operator==(const PlanIdentity & other) const
  {return goal_id == other.goal_id && plan_generation == other.plan_generation;}
  bool operator!=(const PlanIdentity & other) const {return !(*this == other);}
};

// 分段标识只用于进度显示
// 控制器始终使用整条规划的标识
struct SegmentIdentity : PlanIdentity
{
  size_t segment_index{0};
  bool operator==(const SegmentIdentity & other) const
  {return PlanIdentity::operator==(other) && segment_index == other.segment_index;}
  bool operator!=(const SegmentIdentity & other) const {return !(*this == other);}
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
};

// Owned exclusively by the ROS callback thread. All deadlines use monotonic time.
class TaskManager
{
public:
  using Clock = std::chrono::steady_clock;
  explicit TaskManager(double goal_tolerance = 0.02, double retry_delay = 1.0,
    double replan_cooldown = 2.0);
  bool setGoal(const geometry_msgs::msg::PoseStamped & goal);
  bool replan();
  void cancel();
  std::optional<PlanningRequest> nextRequest(bool map_ready, Clock::time_point now);
  bool accept(PlanningResult result, Clock::time_point now);
  bool executionFailed(PlanIdentity identity, const std::string & error, Clock::time_point now);
  bool executionSucceeded(PlanIdentity identity);
  bool observeProgress(PlanIdentity identity, const geometry_msgs::msg::PoseStamped & pose,
    double entry_clearance);
  bool routeInvalid(const ReplanEvent & event, Clock::time_point now);
  bool isPlanConfirmed(PlanIdentity identity) const;
  PlanIdentity identity() const {return identity_;}
  uint64_t generation() const {return identity_.plan_generation;}
  uint64_t replanCount() const {return replan_count_;}
  ReplanReason lastReplanReason() const {return last_replan_reason_;}
  TaskState state() const {return state_;}
  const topology_global_planner::TopologyPlan & plan() const {return plan_;}
  const topology_global_planner::Segment * currentSegment() const;
  SegmentIdentity segmentIdentity() const {return SegmentIdentity{identity_, segment_index_};}
  const std::string & error() const {return error_;}
  static const char * stateName(TaskState state);
  static const char * reasonName(ReplanReason reason);

private:
  void resetProgress();
  double goal_tolerance_;
  Clock::duration retry_delay_;
  Clock::time_point retry_at_{};
  Clock::duration replan_cooldown_;
  Clock::time_point next_route_plan_at_{};
  bool route_replan_pending_{false};  // Dispatch cause, not a second task state.
  PlanIdentity identity_;
  uint64_t replan_count_{0};
  ReplanReason last_replan_reason_{ReplanReason::NONE};
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  TaskState state_{TaskState::IDLE};
  topology_global_planner::TopologyPlan plan_;
  size_t segment_index_{0};
  std::vector<double> path_arcs_, segment_ends_;
  double route_progress_{0.0};
  double connector_progress_{-std::numeric_limits<double>::infinity()};
  std::optional<geometry_msgs::msg::Point> previous_position_;
  // Physical commitment survives goal replacement, cancellation and planning retries.
  std::optional<topology_global_planner::Segment> active_connector_;
  std::string error_;
};
}  // namespace nav_executor
