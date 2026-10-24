#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "map_server/distance_field.hpp"
#include "map_server/map_view.hpp"
#include "map_server/footprint.hpp"
#include "minco_controller/mpc_types.hpp"

namespace minco_controller
{
namespace collision_geometry
{
using Point = Eigen::Vector2d;
struct Separation
{
  double squared_distance{std::numeric_limits<double>::infinity()};
  Point polygon{0, 0}, obstacle{0, 0};
};
inline double cross(const Point & a, const Point & b) {return a.x() * b.y() - a.y() * b.x();}
inline Point project(const Point & p, const Point & a, const Point & b)
{
  const Point d = b - a;
  const double t = d.squaredNorm() > 1e-24 ?
    std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
  return a + t * d;
}
inline void update(Separation & result, const Point & p, const Point & q)
{
  const double squared = (p - q).squaredNorm();
  if (squared < result.squared_distance) {result = {squared, p, q};}
}
inline void segments(Separation & result, const Point & a, const Point & b,
  const Point & c, const Point & d)
{
  const Point ab = b - a, cd = d - c;
  const double denominator = cross(ab, cd);
  if (std::abs(denominator) > 1e-20) {
    const double t = cross(c - a, cd) / denominator;
    const double u = cross(c - a, ab) / denominator;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
      const Point intersection = a + t * ab;
      update(result, intersection, intersection);
      return;
    }
  }
  update(result, a, project(a, c, d));
  update(result, b, project(b, c, d));
  update(result, project(c, a, b), c);
  update(result, project(d, a, b), d);
}
// 凸包内点判定兼容顺/逆时针；退化为线段时交由线段距离处理。
inline bool contains(const std::vector<Point> & polygon, const Point & point)
{
  bool positive = false, negative = false;
  double area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    area += cross(a, b);
    const double side = cross(b - a, point - a);
    positive = positive || side > 1e-14;
    negative = negative || side < -1e-14;
  }
  return std::abs(area) > 1e-18 && !(positive && negative);
}
// 连续凸多边形/线段到闭栅格方块的最近点，覆盖擦角和完全包含。
inline Separation square(const std::vector<Point> & polygon, const Point & lower, double size)
{
  Separation result;
  const std::array<Point, 4> corners{{lower, lower + Point(size, 0),
    lower + Point(size, size), lower + Point(0, size)}};
  for (const auto & point : polygon) {
    const Point nearest = point.cwiseMax(lower).cwiseMin(lower + Point::Constant(size));
    update(result, point, nearest);
  }
  for (const auto & corner : corners) {
    if (contains(polygon, corner)) {update(result, corner, corner); return result;}
  }
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Point & a = polygon[i];
    const Point & b = polygon[(i + 1) % polygon.size()];
    const Point direction = b - a;
    // 线段裁剪直接判断穿过栅格的情形，避免对四条栅格边反复求最近点。
    double first = 0.0, last = 1.0;
    bool intersects = true;
    for (int axis = 0; axis < 2; ++axis) {
      if (std::abs(direction[axis]) < 1e-20) {
        if (a[axis] < lower[axis] || a[axis] > lower[axis] + size) {intersects = false;}
      } else {
        double lo = (lower[axis] - a[axis]) / direction[axis];
        double hi = (lower[axis] + size - a[axis]) / direction[axis];
        if (lo > hi) {std::swap(lo, hi);}
        first = std::max(first, lo); last = std::min(last, hi);
      }
    }
    if (intersects && first <= last) {
      const Point hit = a + first * direction;
      update(result, hit, hit); return result;
    }
    // 非相交多边形最近点必有一侧是顶点；多边形顶点到栅格已在上面处理。
    for (const auto & corner : corners) {update(result, project(corner, a, b), corner);}
  }
  return result;
}

inline std::vector<Point> hull(std::vector<Point> points)
{
  std::sort(points.begin(), points.end(), [](const auto & a, const auto & b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
  points.erase(std::unique(points.begin(), points.end(), [](const auto & a, const auto & b) {
      return a == b;
    }), points.end());
  if (points.size() <= 2) {return points;}
  std::vector<Point> result;
  for (const auto & point : points) {
    while (result.size() >= 2 && cross(result.back() - result[result.size() - 2],
      point - result.back()) <= 0) {result.pop_back();}
    result.push_back(point);
  }
  const size_t lower = result.size();
  for (size_t i = points.size() - 1; i > 0; --i) {
    const auto & point = points[i - 1];
    while (result.size() > lower && cross(result.back() - result[result.size() - 2],
      point - result.back()) <= 0) {result.pop_back();}
    result.push_back(point);
  }
  result.pop_back();
  return result;
}

// 凸轮廓直接使用；凹轮廓耳切为凸三角形，保留凹口而非用整体凸包填平。
inline std::vector<std::vector<Point>> convexParts(const map_server::Footprint & footprint)
{
  std::vector<Point> polygon;
  for (const auto & p : footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw std::invalid_argument("车体轮廓含无效坐标");
    }
    const Point point(p.x, p.y);
    if (polygon.empty() || (point - polygon.back()).squaredNorm() > 1e-24) {
      polygon.push_back(point);
    }
  }
  if (polygon.size() > 1 && (polygon.front() - polygon.back()).squaredNorm() < 1e-24) {
    polygon.pop_back();
  }
  bool simplified = true;
  while (simplified && polygon.size() > 3) {
    simplified = false;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const Point a = polygon[i] - polygon[(i + polygon.size() - 1) % polygon.size()];
      const Point b = polygon[(i + 1) % polygon.size()] - polygon[i];
      if (std::abs(cross(a, b)) < 1e-14 && a.dot(b) >= 0) {
        polygon.erase(polygon.begin() + i); simplified = true; break;
      }
    }
  }
  if (polygon.size() < 3) {throw std::invalid_argument("车体轮廓至少需要三个有效顶点");}
  double area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    area += cross(polygon[i], polygon[(i + 1) % polygon.size()]);
    for (size_t j = i + 1; j < polygon.size(); ++j) {
      if (j == i + 1 || (i == 0 && j == polygon.size() - 1)) {continue;}
      Separation separation;
      segments(separation, polygon[i], polygon[(i + 1) % polygon.size()],
        polygon[j], polygon[(j + 1) % polygon.size()]);
      if (separation.squared_distance < 1e-24) {
        throw std::invalid_argument("车体轮廓的边不能相互交叉");
      }
    }
  }
  if (std::abs(area) < 1e-14) {throw std::invalid_argument("车体轮廓面积为零");}
  if (area < 0) {std::reverse(polygon.begin(), polygon.end());}
  bool convex = true;
  for (size_t i = 0; i < polygon.size(); ++i) {
    convex = convex && cross(polygon[(i + 1) % polygon.size()] - polygon[i],
      polygon[(i + 2) % polygon.size()] - polygon[(i + 1) % polygon.size()]) >= -1e-14;
  }
  if (convex) {return {polygon};}
  std::vector<std::vector<Point>> result;
  while (polygon.size() > 3) {
    bool found = false;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const size_t previous = (i + polygon.size() - 1) % polygon.size();
      const size_t next = (i + 1) % polygon.size();
      const std::vector<Point> triangle{polygon[previous], polygon[i], polygon[next]};
      if (cross(triangle[1] - triangle[0], triangle[2] - triangle[1]) <= 1e-14) {continue;}
      bool occupied = false;
      for (size_t j = 0; j < polygon.size(); ++j) {
        if (j == previous || j == i || j == next) {continue;}
        const auto & q = polygon[j];
        if (cross(triangle[1] - triangle[0], q - triangle[0]) >= -1e-14 &&
          cross(triangle[2] - triangle[1], q - triangle[1]) >= -1e-14 &&
          cross(triangle[0] - triangle[2], q - triangle[2]) >= -1e-14)
        {occupied = true; break;}
      }
      if (occupied) {continue;}
      result.push_back(triangle); polygon.erase(polygon.begin() + i); found = true; break;
    }
    if (!found) {throw std::invalid_argument("无法对车体轮廓进行有效的凸分解");}
  }
  result.push_back(polygon);
  return result;
}
}  // namespace collision_geometry

// EDT 仅提供软代价；所有硬安全约束均由有朝向的 Footprint 和原始障碍栅格生成。
class CostmapCost
{
public:
  CostmapCost(const map_server::MapView & grid, const map_server::Footprint & footprint,
    double collision_margin, double trust_region = 0.15)
  : field_(grid.derivedMaps()), parts_(collision_geometry::convexParts(footprint)),
    margin_(collision_margin), trust_region_(trust_region)
  {
    if (!grid.ready() || !field_ || !std::isfinite(margin_) || margin_ < 0 ||
      !std::isfinite(trust_region_) || trust_region_ <= 0 ||
      grid.geometry().width < 3 || grid.geometry().height < 3)
    {throw std::invalid_argument("Footprint 避障需要有效的地图快照、车体轮廓及安全余量");}
    for (const auto & part : parts_) {
      for (const auto & p : part) {vertex_extent_ = std::max(vertex_extent_, p.norm());}
    }
  }

  float getDistance(double x, double y) const
  {
    map_server::DistanceSample value;
    if (!map_server::sampleDistance(*field_, x, y, value)) {throw std::out_of_range("无法在该位置查询有效障碍距离");}
    return value.distance;
  }
  Eigen::Vector2d getGradient(double x, double y) const
  {
    map_server::DistanceSample value;
    if (!map_server::sampleDistance(*field_, x, y, value)) {throw std::out_of_range("无法在该位置查询有效距离梯度");}
    return {value.dx, value.dy};
  }

  bool sample(const Eigen::Vector2d & position, CostLinearization & result) const
  {
    map_server::DistanceSample value;
    if (!map_server::sampleDistance(*field_, position.x(), position.y(), value)) {return false;}
    const auto penalty = map_server::distancePenalty(value.distance, field_->robot_radius_m,
      field_->inflation_radius_m, field_->cost_scaling_factor);
    result.position = position; result.cost = penalty.cost;
    result.gradient = penalty.derivative * Eigen::Vector2d(value.dx, value.dy);
    result.safety_planes.clear();
    return std::isfinite(result.cost) && result.gradient.allFinite();
  }

  bool sampleMotion(const State & a, const State & b, double dt,
    std::vector<SafetyPlane> & planes) const
  {
    planes.clear();
    if (!std::isfinite(dt) || dt <= 0 || !finite(a) || !finite(b)) {return false;}
    const auto command = motionCommand(a, b, dt);
    const std::vector<MotionVertex> vertices{{{a.x, a.y}, a.yaw, 0}, {{a.x, a.y}, a.yaw, dt}};
    return samplePatches(vertices, command, {a.x, a.y}, dt, planes);
  }

  bool sampleEnvelope(const State & observed, const Control & command, double delay,
    double period, double dt, std::vector<SafetyPlane> & planes) const
  {
    planes.clear();
    const auto vertices = envelopeVertices(observed, delay, period);
    const Eigen::Vector2d delayed = Eigen::Vector2d(observed.x, observed.y) +
      delay * Eigen::Vector2d(observed.vx, observed.vy);
    return samplePatches(vertices, command, delayed, dt, planes);
  }

  void checkPose(const State & state, const std::string & stage) const
  {
    checkMotion(state, state, stage);
  }
  void checkMotion(const State & a, const State & b, const std::string & stage) const
  {
    if (!finite(a) || !finite(b)) {throw std::runtime_error("Footprint 检查收到无效位姿");}
    const std::vector<MotionVertex> vertices{{{a.x, a.y}, a.yaw, 0}, {{a.x, a.y}, a.yaw, 1}};
    checkPatches(vertices, motionCommand(a, b, 1), stage);
  }
  void checkEnvelope(const State & observed, const Control & command, double delay,
    double period, const std::string & stage) const
  {
    checkPatches(envelopeVertices(observed, delay, period), command, stage);
  }

private:
  using Point = collision_geometry::Point;
  struct MotionVertex
  {
    Point origin;
    double heading, time;
  };
  struct Patch
  {
    std::vector<MotionVertex> vertices;
    std::vector<Point> body;
    double error{0};
  };
  static bool finite(const State & s)
  {return Eigen::Vector3d(s.x, s.y, s.yaw).allFinite();}
  static Point rotate(const Point & p, double yaw)
  {return {std::cos(yaw) * p.x() - std::sin(yaw) * p.y(), std::sin(yaw) * p.x() + std::cos(yaw) * p.y()};}
  static MotionVertex midpoint(const MotionVertex & a, const MotionVertex & b)
  {return {0.5 * (a.origin + b.origin), 0.5 * (a.heading + b.heading), 0.5 * (a.time + b.time)};}
  static Control motionCommand(const State & a, const State & b, double duration)
  {return {(b.x - a.x) / duration, (b.y - a.y) / duration, (b.yaw - a.yaw) / duration};}
  static std::vector<MotionVertex> envelopeVertices(const State & s, double delay, double period)
  {
    if (!finite(s) || !Eigen::Vector3d(s.vx, s.vy, s.omega).allFinite() ||
      !std::isfinite(delay) || delay < 0 || !std::isfinite(period) || period <= 0)
    {throw std::invalid_argument("延迟 Footprint 安全检查参数无效");}
    const Point origin(s.x, s.y), delayed = origin + delay * Point(s.vx, s.vy);
    return {{origin, s.yaw, 0}, {origin, s.yaw, delay + period},
      {delayed, s.yaw + delay * s.omega, period}, {delayed, s.yaw + delay * s.omega, 0}};
  }

  // 平移是精确仿射映射；旋转的线性插值误差由 |顶点|*角度跨度^2/8 保守覆盖。
  // vertex_extent_ 只用于这个微小插值误差界，不作为车体碰撞圆半径。
  template<typename Visitor>
  bool patches(const std::vector<MotionVertex> & vertices, const Control & command,
    const Visitor & visitor) const
  {
    if (!Eigen::Vector3d(command.vx, command.vy, command.omega).allFinite()) {return false;}
    std::vector<std::vector<MotionVertex>> pending{vertices};
    size_t work = 0;
    while (!pending.empty()) {
      if (++work > 65536) {throw std::runtime_error("Footprint 转动检查超出计算预算，拒绝控制");}
      auto current = std::move(pending.back()); pending.pop_back();
      double span = 0; size_t first = 0, second = 0;
      for (size_t i = 0; i < current.size(); ++i) {
        for (size_t j = i + 1; j < current.size(); ++j) {
          const double delta = std::abs(current[i].heading - current[j].heading +
            command.omega * (current[i].time - current[j].time));
          if (delta > span) {span = delta; first = i; second = j;}
        }
      }
      if (span > 0.05) {
        if (current.size() == 4) {
          pending.push_back({current[0], current[1], current[2]});
          pending.push_back({current[0], current[2], current[3]});
        } else if (current.size() == 2) {
          const auto middle = midpoint(current[0], current[1]);
          pending.push_back({current[0], middle}); pending.push_back({middle, current[1]});
        } else {
          const size_t other = 3 - first - second;
          const auto middle = midpoint(current[first], current[second]);
          pending.push_back({current[first], middle, current[other]});
          pending.push_back({middle, current[second], current[other]});
        }
        continue;
      }
      for (const auto & part : parts_) {
        Patch patch; patch.vertices = current; patch.error = vertex_extent_ * span * span / 8.0;
        for (const auto & v : current) {
          const Point center = v.origin + v.time * Point(command.vx, command.vy);
          for (const auto & p : part) {patch.body.push_back(center + rotate(p, v.heading + v.time * command.omega));}
        }
        patch.body = collision_geometry::hull(std::move(patch.body));
        if (!visitor(patch, part)) {return false;}
      }
    }
    return true;
  }

  static double minimumProjection(const std::vector<Point> & part, const Point & normal, double yaw)
  {
    double value = std::numeric_limits<double>::infinity();
    const Point local_normal = rotate(normal, -yaw);
    for (const auto & p : part) {value = std::min(value, local_normal.dot(p));}
    return value;
  }
  static void appendPlane(std::vector<SafetyPlane> & planes, SafetyPlane plane)
  {
    for (auto & old : planes) {
      if ((old.normal - plane.normal).squaredNorm() < 1e-20 &&
        std::abs(old.preview_time - plane.preview_time) < 1e-14 &&
        (old.origin_offset - plane.origin_offset).squaredNorm() < 1e-24)
      {old.offset = std::max(old.offset, plane.offset); return;}
    }
    planes.push_back(plane);
  }

  bool samplePatches(const std::vector<MotionVertex> & vertices, const Control & command,
    const Point & base, double dt, std::vector<SafetyPlane> & planes) const
  {
    return patches(vertices, command, [&](const Patch & patch, const std::vector<Point> & part) {
        const double margin = margin_ + patch.error + 0.001;
        const auto add = [&](const Point & normal, double obstacle_projection) {
            for (const auto & vertex : patch.vertices) {
              const double yaw = vertex.heading + vertex.time * command.omega;
              appendPlane(planes, {normal, obstacle_projection + margin -
                minimumProjection(part, normal, yaw), vertex.time - dt, vertex.origin - base});
            }
          };
        add({1, 0}, field_->geometry.origin_x);
        add({-1, 0}, -(field_->geometry.origin_x + field_->geometry.width * field_->geometry.resolution));
        add({0, 1}, field_->geometry.origin_y);
        add({0, -1}, -(field_->geometry.origin_y + field_->geometry.height * field_->geometry.resolution));
        const double reach = margin + std::sqrt(2.0) * trust_region_;
        return visitObstacles(patch.body, reach, [&](int x, int y) {
            const Point lower(field_->geometry.origin_x + x * field_->geometry.resolution, field_->geometry.origin_y + y * field_->geometry.resolution);
            auto separation = collision_geometry::square(patch.body, lower, field_->geometry.resolution);
            if (separation.squared_distance > reach * reach) {return true;}
            if (separation.squared_distance < 1e-20) {
              // 种子移动区域穿过障碍时，从该段起始轮廓选择安全侧，仍由同一 Footprint 决定。
              const auto & v = patch.vertices.front();
              const Point center = v.origin + v.time * Point(command.vx, command.vy);
              std::vector<Point> anchor;
              for (const auto & p : part) {anchor.push_back(center + rotate(p, v.heading + v.time * command.omega));}
              separation = collision_geometry::square(anchor, lower, field_->geometry.resolution);
            }
            if (separation.squared_distance < 1e-20) {return false;}
            const Point normal = (separation.polygon - separation.obstacle).normalized();
            const double support = normal.dot(lower) + field_->geometry.resolution *
              (std::max(0.0, normal.x()) + std::max(0.0, normal.y()));
            add(normal, support);
            return true;
          });
      });
  }

  void checkPatches(const std::vector<MotionVertex> & vertices, const Control & command,
    const std::string & stage) const
  {
    const bool valid = patches(vertices, command, [&](const Patch & patch, const std::vector<Point> &) {
        const double margin = margin_ + patch.error;
        for (const auto & p : patch.body) {
          if (p.x() - margin < field_->geometry.origin_x || p.y() - margin < field_->geometry.origin_y ||
            p.x() + margin >= field_->geometry.origin_x + field_->geometry.width * field_->geometry.resolution ||
            p.y() + margin >= field_->geometry.origin_y + field_->geometry.height * field_->geometry.resolution)
          {throw std::runtime_error(stage + "：车体轮廓及安全余量超出地图范围");}
        }
        return visitObstacles(patch.body, margin, [&](int x, int y) {
            const Point lower(field_->geometry.origin_x + x * field_->geometry.resolution, field_->geometry.origin_y + y * field_->geometry.resolution);
            const auto separation = collision_geometry::square(patch.body, lower, field_->geometry.resolution);
            if (separation.squared_distance <= margin * margin) {
              throw std::runtime_error(stage + "：车体轮廓与" +
                std::string("障碍或未知栅格") +
                "相交或净空不足，轮廓间距=" + std::to_string(std::sqrt(separation.squared_distance)) +
                "米，安全余量=" + std::to_string(margin_) + "米，转动检查误差上界=" +
                std::to_string(patch.error) + "米，栅格索引=(" + std::to_string(x) + "," + std::to_string(y) + ")");
            }
            return true;
          });
      });
    if (!valid) {throw std::runtime_error(stage + "：车体轮廓移动检查失败");}
  }

  template<typename Visitor>
  bool visitObstacles(const std::vector<Point> & polygon, double margin, const Visitor & visitor) const
  {
    Point lower = polygon.front(), upper = lower;
    for (const auto & p : polygon) {lower = lower.cwiseMin(p); upper = upper.cwiseMax(p);}
    const double r = field_->geometry.resolution;
    const auto index = [&](double v, double origin, int count) {
        return static_cast<int>(std::clamp(std::floor((v - origin) / r), 0.0, static_cast<double>(count - 1)));
      };
    const int x0 = std::max(0, index(lower.x() - margin, field_->geometry.origin_x, field_->geometry.width) - 1);
    const int y0 = std::max(0, index(lower.y() - margin, field_->geometry.origin_y, field_->geometry.height) - 1);
    const int x1 = index(upper.x() + margin, field_->geometry.origin_x, field_->geometry.width);
    const int y1 = index(upper.y() + margin, field_->geometry.origin_y, field_->geometry.height);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const auto raw = field_->fused_obstacles.data[static_cast<size_t>(y) * field_->geometry.width + x];
        if ((raw != 0) && !visitor(x, y)) {return false;}
      }
    }
    return true;
  }
  std::shared_ptr<const map_server::DerivedMaps> field_;
  std::vector<std::vector<Point>> parts_;
  double margin_, trust_region_, vertex_extent_{0};
};
}  // namespace minco_controller
