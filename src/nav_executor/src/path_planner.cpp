#include "nav_executor/path_planner.hpp"

#include <exception>
#include <utility>

namespace nav_executor
{
PathPlanner::PathPlanner(Plan plan, Poll poll, std::function<nav_msgs::msg::Path()> original_path)
    : plan_(std::move(plan)), poll_(std::move(poll)), original_path_(std::move(original_path)),
      worker_(&PathPlanner::run, this)
{
}

PathPlanner::~PathPlanner()
{
  stopping_ = true;
  wake_.notify_all();
  worker_.join();
}

void PathPlanner::invalidate(uint64_t generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_ = generation;
  pending_.reset();
  result_.reset();
  wake_.notify_all();
}

void PathPlanner::submit(PlanningRequest request)
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_ = request.identity.plan_generation;
  pending_ = std::move(request);
  result_.reset();
  wake_.notify_all();
}

std::optional<PlanningResult> PathPlanner::takeResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(result_);
  result_.reset();
  return result;
}

void PathPlanner::run()
{
  while (!stopping_) {
    // 规划器的定时器、订阅与规划调用均在同一线程执行
    poll_();
    std::optional<PlanningRequest> request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(20), [this]() {
        return stopping_ || pending_.has_value();
      });
      if (stopping_) {
        return;
      }
      request = std::move(pending_);
      pending_.reset();
    }
    if (!request) {
      continue;
    }
    PlanningResult result;
    result.identity = request->identity;
    const auto cancel = [this, generation = request->identity.plan_generation]() {
      return stopping_ || generation_ != generation;
    };
    try {
      if (!cancel()) {
        result.plan = plan_(*request, cancel);
        result.original_path = original_path_();
      }
    } catch (const std::exception & exception) {
      result.error = exception.what();
    } catch (...) {
      result.error = "Unknown planning exception";
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cancel()) {
      result_ = std::move(result);
    }
  }
}
} // 命名空间 nav_executor
