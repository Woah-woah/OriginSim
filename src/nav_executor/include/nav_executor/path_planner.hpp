#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "nav_executor/task_manager.hpp"

namespace nav_executor
{
// 单个工作线程处理可替换的待规划请求并保存结果
class PathPlanner
{
public:
  using CancelChecker = std::function<bool()>;
  using Plan =
    std::function<topology_global_planner::TopologyPlan(const PlanningRequest &, CancelChecker)>;
  using Poll = std::function<void()>;
  PathPlanner(
    Plan plan,
    Poll poll =
      []() {
      },
    std::function<nav_msgs::msg::Path()> original_path =
      []() {
        return nav_msgs::msg::Path{};
      });
  ~PathPlanner();
  void invalidate(uint64_t generation);
  void submit(PlanningRequest request);
  std::optional<PlanningResult> takeResult();

private:
  void run();
  Plan plan_;
  Poll poll_;
  std::function<nav_msgs::msg::Path()> original_path_;
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> generation_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<PlanningRequest> pending_;
  std::optional<PlanningResult> result_;
  std::thread worker_;
};
} // 命名空间 nav_executor
