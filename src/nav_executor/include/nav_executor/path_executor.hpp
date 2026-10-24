#pragma once

#include <functional>
#include <memory>
#include <string>

#include "nav_executor/task_manager.hpp"
#include "map_server/map_client.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav_executor
{
enum class ExecutionState
{
  IDLE,
  RUNNING,
  STOPPED,
  SUCCEEDED,
  FAILED
};

// 里程计速度以子坐标系原点为参考
geometry_msgs::msg::Twist transformRobotVelocity(const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::TransformStamped & base_from_child);
bool finiteTwist(const geometry_msgs::msg::Twist & speed);

// 接口与控制周期使用互斥回调组，里程计接收和代价地图激活独立运行，里程计快照由互斥锁保护
class PathExecutor
{
public:
  using Failure = std::function<void(PlanIdentity, const std::string &)>;
  using Success = std::function<void(PlanIdentity)>;
  using Confirmation = std::function<bool(PlanIdentity)>;
  using Progress = std::function<void(PlanIdentity, const geometry_msgs::msg::PoseStamped &)>;
  using Observation = std::function<void(
    PlanIdentity, const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &)>;
  PathExecutor(const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<map_server::MapClient> local_costmap,
    std::function<bool()> global_map_ready,
    Failure failure,
    Success success,
    Confirmation confirmation,
    Progress progress,
    Observation observation);
  ~PathExecutor();
  PathExecutor(const PathExecutor &) = delete;
  PathExecutor & operator=(const PathExecutor &) = delete;
  bool acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity);
  void stop(const std::string & reason);
  // 线程安全地锁定急停并发布零速度，无需等待控制器
  void emergencyStop();
  void clearEmergencyStop();
  ExecutionState state() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // 命名空间 nav_executor
