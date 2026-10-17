#pragma once

#include <vector>
#include "minco_controller/mpc_types.hpp"

namespace minco_controller
{
// Piecewise linear geometry with an acceleration-limited scalar speed profile.
// This supplies reference position/velocity/yaw only; it is not a MINCO planner.
class PathTrajectory
{
public:
  void setPath(const std::vector<ReferencePoint> & points,
    double speed, double acceleration, double yaw_rate, bool track_yaw);
  ReferencePoint sample(double time) const;
  double nearestTime(const State & state, double minimum_time = 0.0) const;
  bool empty() const {return knots_.empty();}
  double duration() const {return knots_.empty() ? 0.0 : knots_.back().time;}
  void clear() {knots_.clear();}

private:
  struct Knot
  {
    Eigen::Vector2d position;
    double yaw{0.0}, distance{0.0}, time{0.0}, speed{0.0};
  };
  std::vector<Knot> knots_;
};
}  // namespace minco_controller
