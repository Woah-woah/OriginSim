#include "minco_controller/path_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace minco_controller
{
namespace
{
double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

// MPC 曲率降速
constexpr double kMaxLateralAcceleration = 1.0;  // 最大允许向心加速度，决定弯道速度上限
constexpr double kCurvatureWindow = 0.35;        // 估计曲率时，前后各取多远的路径点
constexpr double kCurvatureEpsilon = 1e-4;       // 小于该曲率就不启用曲率限速
}

void PathTrajectory::setPath(
  const std::vector<ReferencePoint> & points,
  double speed, 
  double acceleration, 
  double yaw_rate, 
  bool track_yaw)
{
  clear();
  if (points.empty() || !std::isfinite(speed) || speed <= 0.0 ||
    !std::isfinite(acceleration) || acceleration <= 0.0 ||
    !std::isfinite(yaw_rate) || yaw_rate <= 0.0)
  {throw std::invalid_argument("局部参考路径或参考运动上限无效");}

  for (const auto & point : points) {
    if (!point.pos.allFinite() || !std::isfinite(point.heading_yaw)) {
      throw std::invalid_argument("局部路径包含无效坐标或朝向");
    }
  }

  knots_.push_back({points.front().pos, track_yaw ? points.front().heading_yaw : 0.0});
  for (size_t i = 1; i < points.size(); ++i) {
    const auto previous = knots_.back();
    const auto delta = points[i].pos - previous.position;
    const double distance = delta.norm();
    const double yaw_delta = track_yaw ? wrap(points[i].heading_yaw - previous.yaw) : 0.0;
    if (distance < 1e-9 && std::abs(yaw_delta) < 1e-9) {continue;}
    const double subdivisions = std::max({2.0, std::ceil(distance / 0.05),
      std::ceil(std::abs(yaw_delta) / 0.1)});
    if (subdivisions + knots_.size() > 100000) {
      throw std::invalid_argument("局部参考路径点数超过十万个，拒绝执行");
    }
    const int count = static_cast<int>(subdivisions);
    for (int j = 1; j <= count; ++j) {
      const double fraction = static_cast<double>(j) / count;
      knots_.push_back({previous.position + fraction * delta,
        previous.yaw + fraction * yaw_delta, previous.distance + fraction * distance});
    }
  }

  const double length = knots_.back().distance;

  // 根据路径累计弧长 s，插值获取对应的二维位置
  const auto positionAt = [this](double s) -> Eigen::Vector2d {
    s = std::clamp(s, 0.0, knots_.back().distance);

    auto it = std::lower_bound(
      knots_.begin(), knots_.end(), s,
      [](const Knot & knot, double distance) {
        return knot.distance < distance;
      });

    if (it == knots_.begin()) {
      return it->position;
    }

    if (it == knots_.end()) {
      return knots_.back().position;
    }

    const auto & previous = *(it - 1);
    const double ds = it->distance - previous.distance;

    if (ds < 1e-9) {
      return it->position;
    }

    const double ratio = (s - previous.distance) / ds;

    return previous.position + ratio * (it->position - previous.position);
  };


  // 根据弧长窗口内的三个点估算路径曲率
  const auto estimateCurvature = [&](double s) -> double {
    const double s0 = std::max(0.0, s - kCurvatureWindow);
    const double s1 = s;
    const double s2 = std::min(length, s + kCurvatureWindow);

    const Eigen::Vector2d p0 = positionAt(s0);
    const Eigen::Vector2d p1 = positionAt(s1);
    const Eigen::Vector2d p2 = positionAt(s2);

    const Eigen::Vector2d a = p1 - p0;
    const Eigen::Vector2d b = p2 - p1;
    const Eigen::Vector2d c = p2 - p0;

    const double denominator = a.norm() * b.norm() * c.norm();

    if (denominator < 1e-8) {
      return 0.0;
    }

    const double cross = a.x() * c.y() - a.y() * c.x();

    return 2.0 * std::abs(cross) / denominator;
  };


  // 加入曲率限速
  for (auto & knot : knots_) {
    knot.speed = std::min(speed, std::sqrt(2.0 * acceleration *
        std::max(0.0, std::min(knot.distance, length - knot.distance))));

    const double curvature = estimateCurvature(knot.distance);

    if (std::isfinite(curvature) && curvature > kCurvatureEpsilon) {
      const double curve_speed = std::sqrt(
        kMaxLateralAcceleration / curvature);

      knot.speed = std::min(knot.speed, curve_speed);
    }
  }

  // 前向加速度约束
  for (size_t i = 1; i < knots_.size(); ++i) {
    auto & a = knots_[i - 1];
    auto & b = knots_[i];
    const double ds = b.distance - a.distance;
    const double dyaw = std::abs(b.yaw - a.yaw);
    if (ds < 1e-9) {a.speed = b.speed = 0.0;}
    else if (dyaw > 1e-9) {
      const double yaw_speed_limit = yaw_rate * ds / dyaw;
      a.speed = std::min(a.speed, yaw_speed_limit);
      b.speed = std::min(b.speed, yaw_speed_limit);
    }
    b.speed = std::min(b.speed, std::sqrt(a.speed * a.speed + 2.0 * acceleration * ds));
  }

  // 后向加速度约束
  for (size_t i = knots_.size(); i > 1; --i) {
    auto & a = knots_[i - 2];
    const auto & b = knots_[i - 1];
    a.speed = std::min(a.speed, std::sqrt(b.speed * b.speed +
        2.0 * acceleration * (b.distance - a.distance)));
  }


  for (size_t i = 1; i < knots_.size(); ++i) {
    const auto & a = knots_[i - 1];
    auto & b = knots_[i];
    const double ds = b.distance - a.distance;
    const double dt = ds > 1e-9 ? 2.0 * ds / (a.speed + b.speed) :
      std::abs(b.yaw - a.yaw) / yaw_rate;
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("局部路径有无法分配有效运行时间的路段");
    }
    b.time = a.time + dt;
  }
}

ReferencePoint PathTrajectory::sample(double time) const
{
  if (empty() || !std::isfinite(time)) {throw std::invalid_argument("局部参考轨迹采样时间无效");}
  ReferencePoint result;
  if (time >= duration() || knots_.size() == 1) {
    result.pos = knots_.back().position;
    result.heading_yaw = knots_.back().yaw;
    for (size_t i = knots_.size(); i > 1; --i) {
      const auto delta = knots_[i - 1].position - knots_[i - 2].position;
      if (delta.squaredNorm() > 1e-18) {
        result.path_tangent_yaw = std::atan2(delta.y(), delta.x());
        break;
      }
    }
    return result;  // Hold the endpoint with zero feedforward velocity.
  }
  time = std::max(0.0, time);
  const auto next = std::upper_bound(knots_.begin(), knots_.end(), time,
    [](double value, const Knot & knot) {return value < knot.time;});
  const auto & a = *(next - 1);
  const auto & b = *next;
  const double dt = b.time - a.time;
  const double elapsed = time - a.time;
  const double ds = b.distance - a.distance;
  const double acceleration = (b.speed - a.speed) / dt;
  const double velocity = a.speed + acceleration * elapsed;
  const double fraction = ds > 1e-9 ?
    std::clamp((a.speed * elapsed + 0.5 * acceleration * elapsed * elapsed) / ds, 0.0, 1.0) :
    elapsed / dt;
  result.pos = a.position + fraction * (b.position - a.position);
  result.heading_yaw = a.yaw + fraction * (b.yaw - a.yaw);
  if (ds > 1e-9) {
    result.path_tangent_yaw = std::atan2(b.position.y() - a.position.y(), b.position.x() - a.position.x());
    result.vel = (b.position - a.position) * (velocity / ds);
    result.yaw_rate = (b.yaw - a.yaw) * velocity / ds;
  } else {
    result.yaw_rate = (b.yaw - a.yaw) / dt;
  }
  return result;
}


double PathTrajectory::nearestTime(const State & state, double minimum_time) const
{
  if (empty()) {throw std::logic_error("局部控制器没有参考路径");}
  double best_distance = std::numeric_limits<double>::infinity();
  double best_time = std::clamp(minimum_time, 0.0, duration());
  const Eigen::Vector2d position(state.x, state.y);
  for (size_t i = 1; i < knots_.size(); ++i) {
    const auto & a = knots_[i - 1];
    const auto & b = knots_[i];
    if (b.time < minimum_time) {continue;}
    const auto delta = b.position - a.position;
    const double ds = b.distance - a.distance;
    double fraction = ds > 1e-9 ? std::clamp((position - a.position).dot(delta) /
        delta.squaredNorm(), 0.0, 1.0) :
      std::clamp(wrap(state.yaw - a.yaw) / (b.yaw - a.yaw), 0.0, 1.0);
    const double squared_distance = (position - a.position - fraction * delta).squaredNorm() +
      (ds > 1e-9 ? 0.0 : 0.01 * std::pow(wrap(state.yaw -
        (a.yaw + fraction * (b.yaw - a.yaw))), 2));
    if (squared_distance < best_distance) {
      const double dt = b.time - a.time;
      const double v = std::sqrt(std::max(0.0, a.speed * a.speed +
          2.0 * (b.speed - a.speed) / dt * fraction * ds));
      const double elapsed = ds > 1e-9 ?
        (fraction * ds > 1e-12 ? 2.0 * fraction * ds / (a.speed + v) : 0.0) : fraction * dt;
      best_time = std::max(minimum_time, a.time + elapsed);
      best_distance = squared_distance;
    }
  }
  return std::min(best_time, duration());
}
}  // namespace minco_controller
