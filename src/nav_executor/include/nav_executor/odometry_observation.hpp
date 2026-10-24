#pragma once

#include <deque>
#include <optional>

#include "nav_msgs/msg/odometry.hpp"

namespace nav_executor
{
inline int64_t observationNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

// 在 TF 和里程计共同覆盖的时间内插值；禁止外推、跨坐标系或跨过期数据插值。
inline std::optional<geometry_msgs::msg::Twist> interpolateOdometryTwist(
  const std::deque<nav_msgs::msg::Odometry> & history, int64_t time_ns, double maximum_gap)
{
  if (history.empty() || time_ns < observationNanoseconds(history.front().header.stamp) ||
    time_ns > observationNanoseconds(history.back().header.stamp))
  {
    return std::nullopt;
  }
  for (size_t i = 0; i < history.size(); ++i) {
    const auto & b = history[i];
    const int64_t tb = observationNanoseconds(b.header.stamp);
    if (tb == time_ns) {return b.twist.twist;}
    if (tb < time_ns || i == 0) {continue;}
    const auto & a = history[i - 1];
    const int64_t ta = observationNanoseconds(a.header.stamp);
    if (tb <= ta || (tb - ta) * 1e-9 > maximum_gap ||
      a.header.frame_id != b.header.frame_id || a.child_frame_id != b.child_frame_id)
    {
      return std::nullopt;
    }
    const double fraction = static_cast<double>(time_ns - ta) / (tb - ta);
    const auto blend = [fraction](double x, double y) {return x + fraction * (y - x);};
    geometry_msgs::msg::Twist speed;
    speed.linear.x = blend(a.twist.twist.linear.x, b.twist.twist.linear.x);
    speed.linear.y = blend(a.twist.twist.linear.y, b.twist.twist.linear.y);
    speed.linear.z = blend(a.twist.twist.linear.z, b.twist.twist.linear.z);
    speed.angular.x = blend(a.twist.twist.angular.x, b.twist.twist.angular.x);
    speed.angular.y = blend(a.twist.twist.angular.y, b.twist.twist.angular.y);
    speed.angular.z = blend(a.twist.twist.angular.z, b.twist.twist.angular.z);
    return speed;
  }
  return std::nullopt;
}
}  // namespace nav_executor
