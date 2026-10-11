#include "nav_executor/path_planner.hpp"

#include <exception>
#include <utility>

namespace nav_executor
{
PathPlanner::PathPlanner(Plan plan, Poll poll)
: plan_(std::move(plan)), poll_(std::move(poll)), worker_(&PathPlanner::run, this) {}

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
    // Planner timers/subscriptions and plan() run on this same worker.
    poll_();
    std::optional<PlanningRequest> request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(20), [this]() {
        return stopping_ || pending_.has_value();
      });
      if (stopping_) {return;}
      request = std::move(pending_);
      pending_.reset();
    }
    if (!request) {continue;}
    PlanningResult result{request->identity, nav_msgs::msg::Path{}, {}};
    const auto cancel = [this, generation = request->identity.plan_generation]() {
        return stopping_ || generation_ != generation;
      };
    try {
      if (!cancel()) {result.path = plan_(*request, cancel);}
    } catch (const std::exception & exception) {
      result.error = exception.what();
    } catch (...) {
      result.error = "Unknown planning exception";
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cancel()) {result_ = std::move(result);}
  }
}
}  // namespace nav_executor
