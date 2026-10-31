#pragma once

#include <chrono>
#include <limits>
#include <vector>

#include "minco_controller/trajectory_safety.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/node.hpp"

namespace nav_executor
{
struct RecoveryConfig
{
  // 用户只配置脱困行为；运动约束与安全参数继承现有导航配置。
  bool enabled{true};
  int max_attempts{3};
  double speed{0.18}, max_distance{1.5}, timeout{30.0}, cooldown{3.0};

  // 以下为内部派生值，不声明独立的 ROS/YAML 参数。
  double total_distance{4.5}, planning_timeout{0.15};
  double stop_timeout{3.0}, settle_time{0.3};
  double acceleration{0.35}, braking{0.35}, velocity_tolerance{0.08};
  double waypoint_tolerance{0.035}, tracking_tolerance{0.15};
  double progress_timeout{3.0}, minimum_progress{0.03};
  double collision_margin{0.02}, delay{0.10};
  static RecoveryConfig load(const rclcpp::Node::SharedPtr & node);
  void validate() const;
};

struct RecoveryCandidate
{
  std::vector<minco_controller::State> states;
  double score{-std::numeric_limits<double>::infinity()};
  double clearance{0}, length{0}, cost{0};
};

struct RecoveryPlan
{
  RecoveryCandidate selected;
  std::vector<RecoveryCandidate> candidates;
  uint64_t producer{0}, version{0}, dynamic_version{0};
};

// 搜索只持有不可变地图；真实轮廓安全判断全部交给公共模块。
class RecoveryPlanner
{
public:
  explicit RecoveryPlanner(RecoveryConfig config);
  RecoveryPlan plan(const map_server::MapView & map, const map_server::Footprint & footprint,
    const minco_controller::State & start, const Eigen::Vector2d & reverse,
    const std::vector<Eigen::Vector2d> & previous_endpoints = {}) const;

private:
  RecoveryConfig config_;
};

nav_msgs::msg::Path recoveryPath(const RecoveryCandidate & candidate,
  const std_msgs::msg::Header & header);
}  // namespace nav_executor
