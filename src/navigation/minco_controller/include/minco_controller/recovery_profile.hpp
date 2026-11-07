#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <utility>

namespace minco_controller
{
// 只在 Recovery 控制线程的一个周期内启用；嵌套计时为包含子调用的累计时间。
enum class RecoveryMetric : size_t
{
  TICK, INPUT, RECOVERY, MPC, SOLVE, QP, PREDICTION, PREDICTED_PATH, FINAL, COLLISION, ESCAPE,
  SAMPLE_MOTION, SAMPLE_ENVELOPE, BRAKING, MOTION, SQUARE, HULL, PROJECTION, APPEND, VISIT, COUNT
};

struct RecoveryTiming
{
  size_t calls{0};
  double ms{0};
};

struct RecoveryProfile
{
  std::array<RecoveryTiming, static_cast<size_t>(RecoveryMetric::COUNT)> timings{};
  size_t obstacle_candidates{0}, plane_comparisons{0}, plane_duplicates{0};
  size_t sample_obstacles{0}, sample_obstacles_max{0}, sample_planes{0}, sample_planes_max{0};
  size_t collision_cache_hits{0};
  double input_age{std::numeric_limits<double>::quiet_NaN()};
  double aligned_age{std::numeric_limits<double>::quiet_NaN()};
  double entry_age{std::numeric_limits<double>::quiet_NaN()};
  double publish_age{std::numeric_limits<double>::quiet_NaN()};
  double ros_begin{0};
  double pose_stamp{std::numeric_limits<double>::quiet_NaN()};
  bool publish_check{false};
  bool published{false};
  bool failed{false};
  const char * stage{"input"};
  const char * phase{"NONE"};
  const RecoveryTiming & get(RecoveryMetric metric) const
  {return timings[static_cast<size_t>(metric)];}
};

inline thread_local RecoveryProfile * active_recovery_profile = nullptr;

inline void recoveryStage(const char * stage)
{
  if (active_recovery_profile) {active_recovery_profile->stage = stage;}
}

class RecoveryTimer
{
public:
  using Clock = std::chrono::steady_clock;
  explicit RecoveryTimer(RecoveryMetric metric)
  : profile_(active_recovery_profile), metric_(metric)
  {
    if (profile_) {began_ = Clock::now();}
  }
  ~RecoveryTimer()
  {
    if (profile_) {
      auto & timing = profile_->timings[static_cast<size_t>(metric_)];
      ++timing.calls;
      timing.ms += std::chrono::duration<double, std::milli>(Clock::now() - began_).count();
    }
  }
  RecoveryTimer(const RecoveryTimer &) = delete;
  RecoveryTimer & operator=(const RecoveryTimer &) = delete;
private:
  RecoveryProfile * profile_;
  RecoveryMetric metric_;
  Clock::time_point began_;
};

// 覆盖提前返回与异常退出，确保超时失败周期仍然输出诊断。
class RecoveryProfileSession
{
public:
  RecoveryProfileSession(bool enabled, std::function<void(const RecoveryProfile &, bool)> report)
  : previous_(active_recovery_profile), enabled_(enabled), report_(std::move(report)),
    exceptions_(std::uncaught_exceptions())
  {
    if (enabled_) {active_recovery_profile = &profile_;}
  }
  ~RecoveryProfileSession()
  {
    active_recovery_profile = previous_;
    if (enabled_) {report_(profile_, std::uncaught_exceptions() > exceptions_);}
  }
private:
  RecoveryProfile profile_;
  RecoveryProfile * previous_;
  bool enabled_;
  std::function<void(const RecoveryProfile &, bool)> report_;
  int exceptions_;
};
}  // namespace minco_controller
