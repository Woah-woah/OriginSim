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
// One worker, one replaceable pending request, one result. No detached threads.
class PathPlanner
{
public:
  using CancelChecker = std::function<bool()>;
  using Plan = std::function<nav_msgs::msg::Path(const PlanningRequest &, CancelChecker)>;
  using Poll = std::function<void()>;
  PathPlanner(Plan plan, Poll poll = []() {});
  ~PathPlanner();
  void invalidate(uint64_t generation);
  void submit(PlanningRequest request);
  std::optional<PlanningResult> takeResult();

private:
  void run();
  Plan plan_;
  Poll poll_;
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> generation_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<PlanningRequest> pending_;
  std::optional<PlanningResult> result_;
  std::thread worker_;
};
}  // namespace nav_executor
