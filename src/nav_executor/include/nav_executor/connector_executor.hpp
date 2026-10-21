#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "topology_global_planner/topology_plan.hpp"

namespace nav_executor
{
struct ConnectorConfig
{
  double region_half_width{0.8};
  double entry_extension{1.0};
  double exit_extension{0.3};
  double exit_hysteresis{0.05};
  double exit_debounce{0.2};
  double pose_timeout{0.5};
};

// 由串行回调管理几何与指令切换，旧区域保留至完整车体轮廓离开
class ConnectorExecutor
{
public:
  using Clock = std::chrono::steady_clock;
  using Command = std::function<void(bool)>;
  using Log = std::function<void(const std::string &)>;
  ConnectorExecutor(ConnectorConfig config,
    std::string frame,
    std::vector<geometry_msgs::msg::Point> footprint,
    Command command,
    Log log = {});
  void setPlan(const topology_global_planner::TopologyPlan & plan);
  void observePose(const geometry_msgs::msg::PoseStamped & pose, Clock::time_point now);
  void localizationLost();

  bool lowered() const
  {
    return lowered_;
  }

  size_t regionCount() const
  {
    return regions_.size();
  }

private:
  struct Region
  {
    std::string id;
    double x;
    double y;
    double ux;
    double uy;
    double length;
    std::vector<geometry_msgs::msg::Point> polygon;
    std::vector<geometry_msgs::msg::Point> exit_polygon;
    bool current{true};
    bool occupied{false};
    std::optional<Clock::time_point> outside_since;
  };

  bool intersects(const Region & region,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    double padding) const;
  ConnectorConfig config_;
  std::string frame_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  Command command_;
  Log log_;
  std::vector<Region> regions_;
  bool lowered_{false};
  std::optional<Clock::time_point> last_pose_at_;
};
} // 命名空间 nav_executor
