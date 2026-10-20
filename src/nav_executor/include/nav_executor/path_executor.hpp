#pragma once

#include <functional>
#include <memory>
#include <string>

#include "nav_executor/task_manager.hpp"
#include "map_server/costmap_client.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav_executor
{
enum class ExecutionState {IDLE, RUNNING, STOPPED, SUCCEEDED, FAILED};

// Odometry twist is expressed at the child-frame origin, not in header.frame_id.
geometry_msgs::msg::Twist transformRobotVelocity(
  const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::TransformStamped & base_from_child);
bool finiteTwist(const geometry_msgs::msg::Twist & speed);

// Peer of PathPlanner. Public methods and control ticks share the node's
// mutually exclusive callback group. Odometry reception and Costmap activation
// run independently; the odometry snapshot is protected by a mutex.
class PathExecutor
{
public:
  using Failure = std::function<void(SegmentIdentity, const std::string &)>;
  using Success = std::function<void(SegmentIdentity)>;
  using Confirmation = std::function<bool(SegmentIdentity)>;
  using Observation = std::function<void(SegmentIdentity,
      const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &)>;
  PathExecutor(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<map_server::CostmapClient> local_costmap,
    std::function<bool()> global_map_ready, Failure failure, Success success,
    Confirmation confirmation, Observation observation);
  ~PathExecutor();
  PathExecutor(const PathExecutor &) = delete;
  PathExecutor & operator=(const PathExecutor &) = delete;
  bool acceptSegment(const topology_global_planner::Segment & segment,
    SegmentIdentity identity, bool final_segment, bool continuous);
  void stop(const std::string & reason);
  // Thread-safe: latches inhibition and publishes zero without waiting for MPC.
  void emergencyStop();
  void clearEmergencyStop();
  ExecutionState state() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace nav_executor
