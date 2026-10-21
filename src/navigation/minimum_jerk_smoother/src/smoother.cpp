#include "minimum_jerk_smoother/smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "map_server/costmap_layers.hpp"
#include "tdt/minimumSnap.hpp"

namespace minimum_jerk_smoother
{
namespace
{
using Plan = topology_global_planner::TopologyPlan;
using Segment = topology_global_planner::Segment;
using Type = topology_global_planner::SegmentType;
using Pose = geometry_msgs::msg::PoseStamped;
using Clock = std::chrono::steady_clock;

struct Axis
{
  double x, y, ux, uy, length;
  explicit Axis(const Segment & segment)
  {
    x = segment.wait_pose.pose.position.x;
    y = segment.wait_pose.pose.position.y;
    const auto & end = segment.exit_pose.pose.position;
    length = std::hypot(end.x - x, end.y - y);
    if (!std::isfinite(length) || length <= 1e-6) {
      throw std::invalid_argument("Invalid connector axis");
    }
    ux = (end.x - x) / length;
    uy = (end.y - y) / length;
  }
  double along(const Pose & pose) const
  {
    const auto & p = pose.pose.position;
    return (p.x - x) * ux + (p.y - y) * uy;
  }
  double lateral(const Pose & pose) const
  {
    const auto & p = pose.pose.position;
    return std::abs((p.y - y) * ux - (p.x - x) * uy);
  }
};

// 精确插入入口/出口截面交点，两个 Segment 共享同一个位姿和朝向。
size_t crossing(std::vector<Pose> & points, size_t begin, const Axis & axis, double plane)
{
  if (std::abs(axis.along(points.at(begin)) - plane) < 1e-7) {
    return begin;
  }
  for (size_t i = begin + 1; i < points.size(); ++i) {
    const double a = axis.along(points[i - 1]), b = axis.along(points[i]);
    if (a <= plane && b >= plane && b - a > 1e-9) {
      const double f = (plane - a) / (b - a);
      auto pose = points[i - 1];
      pose.pose.position.x += f * (points[i].pose.position.x - pose.pose.position.x);
      pose.pose.position.y += f * (points[i].pose.position.y - pose.pose.position.y);
      points.insert(points.begin() + i, pose);
      return i;
    }
  }
  throw std::runtime_error("Trajectory missed a directed connector section");
}

Plan split(const Plan & original, nav_msgs::msg::Path curve, double radius)
{
  Plan result;
  result.full_path.header = curve.header;
  size_t begin = 0;
  for (size_t i = 0; i < original.segments.size(); ++i) {
    auto segment = original.segments[i];
    size_t end = curve.poses.size() - 1;
    if (segment.type == Type::CONNECTOR) {
      const Axis axis(segment);
      if (!std::isfinite(segment.corridor_half_width) || segment.corridor_half_width <= radius) {
        throw std::runtime_error("Connector is narrower than the robot footprint");
      }
      end = crossing(curve.poses, begin, axis, axis.length);
      double previous = axis.along(curve.poses[begin]);
      const double original_begin = axis.along(segment.path.poses.front());
      for (size_t j = begin; j <= end; ++j) {
        const double along = axis.along(curve.poses[j]);
        // 车体外接圆必须完全位于通道边界内，禁止逆向及越过入口后回退。
        if (along < std::min(0.0, original_begin) - 1e-5 || along < previous - 1e-6 ||
            axis.lateral(curve.poses[j]) + radius > segment.corridor_half_width) {
          throw std::runtime_error("Trajectory violates directed connector body clearance");
        }
        previous = along;
      }
    } else if (i + 1 < original.segments.size()) {
      if (original.segments[i + 1].type != Type::CONNECTOR) {
        throw std::runtime_error("Unexpected adjacent NORMAL segments");
      }
      end = crossing(curve.poses, begin, Axis(original.segments[i + 1]), 0.0);
    }
    segment.path.header = curve.header;
    segment.path.poses.assign(curve.poses.begin() + begin, curve.poses.begin() + end + 1);
    result.full_path.poses.insert(
      result.full_path.poses.end(), segment.path.poses.begin(), segment.path.poses.end());
    result.segments.push_back(std::move(segment));
    begin = end;
  }
  if (begin + 1 != curve.poses.size()) {
    throw std::runtime_error("Unassigned trajectory suffix");
  }
  return result;
}
}  // namespace

Smoother::Smoother(Config config) : config_(config)
{
  for (double value : {config.timeout, config.control_spacing, config.corridor_range,
         config.max_speed, config.max_acceleration}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Path smoother numerical parameters must be finite and positive");
    }
  }
  if (config.collision_iterations < 1 || config.collision_iterations > 20 ||
      config.topology_iterations < 1 || config.topology_iterations > 10) {
    throw std::invalid_argument("Path smoother iteration count is out of range");
  }
}

Result Smoother::smooth(const Plan & original, const map_server::Costmap2D & map,
  double body_radius, Validate topology_valid, Cancel cancelled) const
{
  Result fallback{original, false, "disabled"};
  if (!config_.enabled) {
    return fallback;
  }
  const auto deadline = Clock::now() + std::chrono::duration<double>(config_.timeout);
  auto interrupted = [&]() { return cancelled() || Clock::now() >= deadline; };
  fallback.reason = "Optimization failed or exceeded its deadline";
  try {
    if (!std::isfinite(body_radius) || body_radius <= 0 || !topology_valid ||
        original.full_path.poses.size() < 2 || original.segments.empty() ||
        map.getSizeInCellsX() < 3 || map.getSizeInCellsY() < 3 || interrupted()) {
      return fallback;
    }
    for (const auto & segment : original.segments) {
      if (segment.path.poses.empty()) {
        fallback.reason = "Original plan contains an empty segment";
        return fallback;
      }
    }
    // 与前端共用地图语义；额外膨胀覆盖障碍栅格面积和多项式采样间隙。
    auto safe = map;
    const double resolution = map.getResolution();
    const double padding = body_radius + 2.0 * resolution;
    map_server::InflationOptions inflation;
    inflation.radius = padding;
    inflation.inscribed_radius = padding;
    inflation.inflate_around_unknown = true;
    inflation.inflate_unknown = true;
    map_server::inflate(safe, inflation);
    tdt::MinimumSnap::Map binary(safe.getSizeInCellsY(), safe.getSizeInCellsX());
    for (unsigned int y = 0; y < safe.getSizeInCellsY(); ++y) {
      if (interrupted()) {
        return fallback;
      }
      for (unsigned int x = 0; x < safe.getSizeInCellsX(); ++x) {
        const double wx = safe.getOriginX() + (x + 0.5) * resolution;
        const double wy = safe.getOriginY() + (y + 0.5) * resolution;
        const bool border =
          wx - safe.getOriginX() < padding || wy - safe.getOriginY() < padding ||
          safe.getOriginX() + safe.getSizeInCellsX() * resolution - wx < padding ||
          safe.getOriginY() + safe.getSizeInCellsY() * resolution - wy < padding;
        binary(y, x) =
          !border && safe.getCost(x, y) < map_server::INSCRIBED_INFLATED_OBSTACLE ? 255 : 0;
      }
    }
    tdt::MinimumSnap solver;
    solver.setMap(binary, resolution, safe.getOriginX(), safe.getOriginY());
    solver.setOrder(6);  // 六个系数，即五次多项式。
    solver.setMaxDx(3);  // 最小化 jerk，跨全部 NORMAL/CONNECTOR 保证 C2 连续。
    solver.setStrictCollision(true);
    solver.setDt(resolution * 0.25);
    solver.setCancelChecker(interrupted);
    // 按原始折线弧长选点，保留转角；只去掉重复位置及短共线边。
    std::vector<Eigen::Vector2f> points;
    for (const auto & pose : original.full_path.poses) {
      const auto & p = pose.pose.position;
      Eigen::Vector2f next(p.x, p.y);
      if (!next.allFinite()) {
        return fallback;
      }
      if (!points.empty() && (next - points.back()).norm() < 1e-5) {
        continue;
      }
      if (points.size() >= 2) {
        const auto a = points.back() - points[points.size() - 2];
        const auto b = next - points.back();
        if (a.dot(b) > 0 && std::abs(a.x() * b.y() - a.y() * b.x()) < 1e-4 * a.norm() * b.norm() &&
            (next - points[points.size() - 2]).norm() < config_.control_spacing) {
          points.pop_back();
        }
      }
      points.push_back(next);
    }
    if (points.size() < 2 || points.size() > 100000 || interrupted()) {
      return fallback;
    }
    // Douglas-Peucker 简化保留拐角，随后按距离补点，控制稠密 QP 的规模。
    std::vector<bool> keep(points.size(), false);
    keep.front() = keep.back() = true;
    std::function<void(size_t, size_t, int)> simplify = [&](size_t first, size_t last, int depth) {
      if (depth > 48) {
        throw std::runtime_error("Path simplification budget exceeded");
      }
      if (last <= first + 1 || interrupted()) {
        return;
      }
      const Eigen::Vector2f delta = points[last] - points[first];
      double maximum = 0.;
      size_t selected = (first + last) / 2;
      for (size_t j = first + 1; j < last; ++j) {
        const double f = delta.squaredNorm() > 1e-12
                           ? std::clamp(static_cast<double>((points[j] - points[first]).dot(delta) /
                                                            delta.squaredNorm()),
                               0.0, 1.0)
                           : 0.0;
        const double distance = (points[j] - points[first] - f * delta).norm();
        if (distance > maximum) {
          maximum = distance;
          selected = j;
        }
      }
      if (maximum > std::min(0.04, config_.control_spacing * 0.1) ||
          solver.lineInObsticle(points[first], points[last])) {
        keep[selected] = true;
        simplify(first, selected, depth + 1);
        simplify(selected, last, depth + 1);
      }
    };
    simplify(0, points.size() - 1, 0);
    std::vector<Eigen::Vector2f> controls{points.front()};
    for (size_t i = 1; i < points.size(); ++i) {
      if (!keep[i]) {
        continue;
      }
      const Eigen::Vector2f start = controls.back();
      const double length = (points[i] - start).norm();
      const size_t count =
        std::max(1, static_cast<int>(std::ceil(length / config_.control_spacing)));
      if (controls.size() + count > 256) {
        fallback.reason = "Control point budget exceeded";
        return fallback;
      }
      for (size_t j = 1; j <= count; ++j) {
        controls.push_back(start + (points[i] - start) * (static_cast<float>(j) / count));
      }
    }
    points = std::move(controls);
    for (int attempt = 0; attempt < config_.topology_iterations && !interrupted(); ++attempt) {
      tdt::MinimumSnap::SolveInput input;
      input.setPath(points);
      input.setBackend(tdt::MinimumSnap::Backend::OSQPCorridor);
      input.setNormTime(true);
      input.setMaxSpeed(config_.max_speed);
      input.setMaxAcc(config_.max_acceleration);
      input.setCollisionCheckIter(config_.collision_iterations);
      input.setMaxCorridorRange(config_.corridor_range * std::pow(0.5, attempt));
      solver.setTL(std::max(1e-6, std::chrono::duration<double>(deadline - Clock::now()).count()));
      const auto output = solver.solve(input);
      if (!output.success || output.path.size() < 2 || interrupted()) {
        if (!solver.getLastError().empty()) {
          fallback.reason = solver.getLastError();
        }
        continue;
      }
      nav_msgs::msg::Path curve;
      curve.header = original.full_path.header;
      for (size_t j = 0; j < output.path.size(); ++j) {
        Pose pose;
        pose.header = curve.header;
        pose.pose.position.x = output.path[j].x();
        pose.pose.position.y = output.path[j].y();
        const auto delta = j + 1 < output.path.size() ? output.path[j + 1] - output.path[j]
                                                      : output.path[j] - output.path[j - 1];
        const double yaw = std::atan2(delta.y(), delta.x());
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        curve.poses.push_back(pose);
      }
      curve.poses.front() = original.full_path.poses.front();
      curve.poses.back() = original.full_path.poses.back();
      // 端点精确恢复后再完整复检；不允许采样或浮点误差绕过地图检查。
      bool clear = true;
      for (size_t j = 1; j < curve.poses.size() && clear; ++j) {
        const auto & a = curve.poses[j - 1].pose.position;
        const auto & b = curve.poses[j].pose.position;
        clear = !solver.lineInObsticle(Eigen::Vector2f(a.x, a.y), Eigen::Vector2f(b.x, b.y));
        if (j % 128 == 0 && interrupted()) {
          return fallback;
        }
      }
      if (!clear) {
        continue;
      }
      try {
        auto candidate = split(original, std::move(curve), body_radius + resolution * 0.25);
        if (topology_valid(candidate) && !interrupted()) {
          return {std::move(candidate), true, {}};
        }
        fallback.reason = "Smoothed path violates topology";
      } catch (const std::exception & error) {
        fallback.reason = error.what();
      }
    }
  } catch (const std::exception & error) {
    fallback.reason = error.what();
  }
  if (cancelled()) {
    fallback.reason = "Smoothing cancelled";
  } else if (Clock::now() >= deadline) {
    fallback.reason = "Smoothing timed out";
  }
  return fallback;
}
}  // namespace minimum_jerk_smoother
