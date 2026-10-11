// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#ifndef MPPI_CONTROLLER__GOAL_CHECKER_HPP_
#define MPPI_CONTROLLER__GOAL_CHECKER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mppi
{
// Optional caller-owned tolerance provider. No Controller Server or plugin dependency.
class GoalChecker
{
public:
  virtual ~GoalChecker() = default;
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & velocity_tolerance) = 0;
};
}  // namespace mppi
#endif  // MPPI_CONTROLLER__GOAL_CHECKER_HPP_
