#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "map_server/distance_field.hpp"
#include "map_server/map_view.hpp"
#include "map_server/footprint.hpp"
#include "minco_controller/mpc_types.hpp"
#include "minco_controller/recovery_profile.hpp"

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
  RecoveryTimer metric_timer(RecoveryMetric::SQUARE);
  Separation result;
  const std::array<Point, 4> corners{{lower, lower + Point(size, 0),
    lower + Point(size, size), lower + Point(0, size)}};
  for (const auto & point : polygon) {
    const Point nearest = point.cwiseMax(lower).cwiseMin(lower + Point::Constant(size));
    update(result, point, nearest);
    // 第一个零距离已经是全局最小值；后续 update 使用严格小于，不可能再改变最近点。
    if (result.squared_distance == 0.0) {return result;}
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
  RecoveryTimer metric_timer(RecoveryMetric::HULL);
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
class TrajectorySafety
{
public:
  TrajectorySafety(const map_server::MapView & grid, const map_server::Footprint & footprint,
    double collision_margin, double trust_region = 0.15)
  : grid_(grid), field_(grid.derivedMaps()), parts_(collision_geometry::convexParts(footprint)),
    margin_(collision_margin), trust_region_(trust_region)
  {
    if (!grid.ready() || !field_ || !std::isfinite(margin_) || margin_ < 0 ||
      !std::isfinite(trust_region_) || trust_region_ <= 0 ||
      grid.geometry().width < 3 || grid.geometry().height < 3)
    {throw std::invalid_argument("Footprint 避障需要有效的地图快照、车体轮廓及安全余量");}
    // 已配对的动态几何完整包含在 fused_obstacles；仅不配对时额外查询最新层。
    // 这避免每个预测面片对同一批点云重复做栅格投影，不改变碰撞几何。
    unpaired_dynamic_ = grid.dynamicObstacles() &&
      grid.derived().dynamic_version != grid.dynamicVersion();
    for (const auto & part : parts_) {
      for (const auto & p : part) {vertex_extent_ = std::max(vertex_extent_, p.norm());}
    }
  }

  // 返回连续扫掠轮廓到闭障碍栅格的保守净空，截断仅影响评分。
  double clearance(const State & a, const State & b, double cap = 1.0) const
  {
    double result = cap;
    const std::vector<MotionVertex> vertices{{{a.x, a.y}, a.yaw, 0}, {{a.x, a.y}, a.yaw, 1}};
    if (!patches(vertices, motionCommand(a, b, 1), [&](const Patch & patch, const auto &) {
      for (const auto & p : patch.body) {
        result = std::min({result, p.x() - field_->geometry.origin_x,
          p.y() - field_->geometry.origin_y,
          field_->geometry.origin_x + field_->geometry.width * field_->geometry.resolution - p.x(),
          field_->geometry.origin_y + field_->geometry.height * field_->geometry.resolution - p.y()});
      }
      return visitObstacles(patch.body, cap, [&](int x, int y) {
        const Point lower(field_->geometry.origin_x + x * field_->geometry.resolution,
          field_->geometry.origin_y + y * field_->geometry.resolution);
        result = std::min(result, std::sqrt(collision_geometry::square(
          patch.body, lower, field_->geometry.resolution).squared_distance) - patch.error);
        return true;
      });
    })) {throw std::runtime_error("净空计算收到无效运动");}
    return result - margin_;
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

  bool sampleMotion(
    const State & a,
    const State & b,
    double dt,
    std::vector<SafetyPlane> & planes,
    const std::unordered_set<size_t> * initial_contacts = nullptr) const
  {
    RecoveryTimer timer(RecoveryMetric::SAMPLE_MOTION);
    planes.clear();
    if (!std::isfinite(dt) || dt <= 0 || !finite(a) || !finite(b)) {return false;}
    const auto command = motionCommand(a, b, dt);
    const std::vector<MotionVertex> vertices{{{a.x, a.y}, a.yaw, 0}, {{a.x, a.y}, a.yaw, dt}};
    const size_t before = active_recovery_profile ? active_recovery_profile->obstacle_candidates : 0;
    const bool valid = samplePatches(vertices, command, {a.x, a.y}, dt, planes, initial_contacts);
    if (auto * profile = active_recovery_profile) {
      const size_t visited = profile->obstacle_candidates - before;
      profile->sample_obstacles += visited;
      profile->sample_obstacles_max = std::max(profile->sample_obstacles_max, visited);
      profile->sample_planes += planes.size();
      profile->sample_planes_max = std::max(profile->sample_planes_max, planes.size());
    }
    return valid;
  }

  bool sampleEnvelope(
    const State & observed,
    const Control & command,
    double delay,
    double period,
    double dt,
    std::vector<SafetyPlane> & planes,
    const std::unordered_set<size_t> * initial_contacts = nullptr) const
  {
    RecoveryTimer timer(RecoveryMetric::SAMPLE_ENVELOPE);
    planes.clear();
    const auto vertices = envelopeVertices(observed, delay, period);
    const Eigen::Vector2d delayed = Eigen::Vector2d(observed.x, observed.y) +
      delay * Eigen::Vector2d(observed.vx, observed.vy);
    return samplePatches(vertices, command, delayed, dt, planes, initial_contacts);
  }

  // 返回实际轮廓接触的唯一障碍栅格数；无效位姿或越界返回 -1。
  int collisionCellCount(const State & pose,
    std::unordered_set<size_t> * contact_cells = nullptr) const
  {
    RecoveryTimer timer(RecoveryMetric::COLLISION);
    if (contact_cells) {contact_cells->clear();}
    if (!finite(pose)) {return -1;}
    // 缓存仅属于本对象冻结的地图和轮廓，使用精确坐标键，不进行近似匹配。
    const std::array<double, 3> key{{pose.x, pose.y, pose.yaw}};
    const auto cached = collision_cache_.find(key);
    if (cached != collision_cache_.end()) {
      if (active_recovery_profile) {++active_recovery_profile->collision_cache_hits;}
      if (contact_cells) {*contact_cells = cached->second;}
      return static_cast<int>(cached->second.size());
    }
    const auto & g = field_->geometry;
    const double res = g.resolution;
    std::unordered_set<size_t> cells;
    const std::vector<MotionVertex> vertices{
      {{pose.x, pose.y}, pose.yaw, 0}, {{pose.x, pose.y}, pose.yaw, 1}};
    const bool valid = patches(vertices, Control{}, [&](const Patch & patch, const auto &) {
        for (const auto & p : patch.body) {
          if (p.x() - margin_ < g.origin_x || p.y() - margin_ < g.origin_y ||
            p.x() + margin_ >= g.origin_x + g.width * res ||
            p.y() + margin_ >= g.origin_y + g.height * res)
          {return false;}
        }
        return visitObstacles(patch.body, margin_, [&](int x, int y) {
            const Point lower(g.origin_x + x * res, g.origin_y + y * res);
            if (collision_geometry::square(patch.body, lower, res).squared_distance <=
              margin_ * margin_)
            {cells.insert(static_cast<size_t>(y) * g.width + x);}
            return true;
          });
      });
    if (!valid) {return -1;}
    if (contact_cells) {*contact_cells = cells;}
    // 限制每个冻结查询对象的内存，不缓存无效位姿；命中与否不影响判断。
    if (collision_cache_.size() < 512) {collision_cache_.emplace(key, cells);}
    return static_cast<int>(cells.size());
  }

  // 采样检查趋势，并用连续扫掠阻止采样点之间接触新的栅格。
  bool checkEscapeMotion(const State & start, const State & end,
    const std::unordered_set<size_t> * initial_contacts = nullptr) const
  {
    RecoveryTimer timer(RecoveryMetric::ESCAPE);
    if (!finite(start) || !finite(end)) {return false;}
    std::unordered_set<size_t> local_contacts;
    int last_count = collisionCellCount(start, &local_contacts);
    if (last_count < 0) {return false;}
    const auto & allowed = initial_contacts ? *initial_contacts : local_contacts;
    for (const auto cell : local_contacts) {
      if (!allowed.count(cell)) {return false;}
    }
    const double distance = std::hypot(end.x - start.x, end.y - start.y) +
      vertex_extent_ * std::abs(end.yaw - start.yaw);
    const double count = std::ceil(distance / std::min(0.025, field_->geometry.resolution * 0.5));
    if (!std::isfinite(count) || count > 10000) {return false;}
    const int steps = std::max(1, static_cast<int>(count));
    State previous = start;
    try {
      for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        State pose = start;
        pose.x += (end.x - start.x) * t;
        pose.y += (end.y - start.y) * t;
        pose.yaw += (end.yaw - start.yaw) * t;
        const int contacts = collisionCellCount(pose);
        if (contacts < 0 || contacts > last_count) {return false;}
        checkMotion(previous, pose, "Recovery 连续脱困检查", last_count ? &allowed : nullptr);
        last_count = contacts;
        previous = pose;
      }
    } catch (const std::runtime_error &) {return false;}
    return true;
  }

  // 覆盖 [0, delay] 的执行延迟；每条时间有序分支都不能恶化接触。
  void checkEscapeEnvelope(const State & observed, const Control & command,
    double delay, double period, const std::unordered_set<size_t> & contacts) const
  {
    checkEnvelope(observed, command, delay, period, "Recovery 指令延迟包络", &contacts);
    const double travel = delay * (std::hypot(observed.vx, observed.vy) +
      std::hypot(command.vx, command.vy) + vertex_extent_ *
      (std::abs(observed.omega) + std::abs(command.omega)));
    const double count = std::ceil(travel / std::min(0.025, field_->geometry.resolution * 0.5));
    if (!std::isfinite(count) || count > 10000) {
      throw std::runtime_error("Recovery 延迟检查超出预算");
    }
    const int steps = std::max(1, static_cast<int>(count));
    for (int i = 0; i <= steps; ++i) {
      const double latency = delay * i / steps;
      State delayed = observed;
      delayed.x += observed.vx * latency;
      delayed.y += observed.vy * latency;
      delayed.yaw += observed.omega * latency;
      State held = delayed;
      held.x += command.vx * (delay + period - latency);
      held.y += command.vy * (delay + period - latency);
      held.yaw += command.omega * (delay + period - latency);
      if (!checkEscapeMotion(observed, delayed, &contacts) ||
        !checkEscapeMotion(delayed, held, &contacts))
      {throw std::runtime_error("Recovery 延迟或指令保持期间接触恶化");}
    }
  }

  // 先保持当前速度等待停车命令生效，再按保守减速度制动；同时覆盖实际转动。
  void checkBrakingMotion(const State & start, double latency, double braking,
    double alpha, const std::unordered_set<size_t> & contacts) const
  {
    RecoveryTimer timer(RecoveryMetric::BRAKING);
    if (!Eigen::Vector3d(start.vx, start.vy, start.omega).allFinite() ||
      !std::isfinite(latency) || latency < 0 || !std::isfinite(braking) || braking <= 0 ||
      !std::isfinite(alpha) || alpha <= 0)
    {throw std::runtime_error("Recovery 制动参数无效");}
    const double speed = std::hypot(start.vx, start.vy);
    const double duration = latency + std::max(speed / braking, std::abs(start.omega) / alpha);
    const double travel = duration * (speed + vertex_extent_ * std::abs(start.omega));
    const double count = std::ceil(travel / std::min(0.025, field_->geometry.resolution * 0.5));
    if (!std::isfinite(count) || count > 10000) {
      throw std::runtime_error("Recovery 制动检查超出预算");
    }
    const int steps = std::max(1, static_cast<int>(count));
    const auto distance = [latency](double t, double velocity, double acceleration) {
        const double coast = std::min(t, latency);
        const double brake = std::min(std::max(0.0, t - latency), std::abs(velocity) / acceleration);
        return velocity * coast + std::copysign(std::abs(velocity) * brake -
          0.5 * acceleration * brake * brake, velocity);
      };
    State previous = start;
    for (int i = 1; i <= steps; ++i) {
      const double t = duration * i / steps;
      State pose = start;
      const double translation = distance(t, speed, braking);
      if (speed > 0) {
        pose.x += start.vx / speed * translation;
        pose.y += start.vy / speed * translation;
      }
      pose.yaw += distance(t, start.omega, alpha);
      if (!checkEscapeMotion(previous, pose, &contacts)) {
        throw std::runtime_error("Recovery 惯性制动接触恶化");
      }
      // 平移和转动的制动时长不同，矩形参数包络覆盖两者的全部组合。
      const std::vector<MotionVertex> vertices{
        {{previous.x, previous.y}, previous.yaw, 0},
        {{pose.x, pose.y}, previous.yaw, 0},
        {{pose.x, pose.y}, pose.yaw, 0},
        {{previous.x, previous.y}, pose.yaw, 0}};
      checkPatches(vertices, Control{}, "Recovery 制动连续包络",
        collisionCellCount(previous) > 0 ? &contacts : nullptr);
      previous = pose;
    }
  }

  void checkPose(const State & state, const std::string & stage) const
  {
    checkMotion(state, state, stage);
  }

  void checkMotion(
    const State & a,
    const State & b,
    const std::string & stage,
    const std::unordered_set<size_t> * allowed_contacts = nullptr) const
  {
    RecoveryTimer timer(RecoveryMetric::MOTION);
    if (!finite(a) || !finite(b)) {throw std::runtime_error("Footprint 检查收到无效位姿");}
    const std::vector<MotionVertex> vertices{{{a.x, a.y}, a.yaw, 0}, {{a.x, a.y}, a.yaw, 1}};
    checkPatches(vertices, motionCommand(a, b, 1), stage, allowed_contacts);
  }
  void checkEnvelope(
    const State & observed,
    const Control & command,
    double delay,
    double period,
    const std::string & stage,
    const std::unordered_set<size_t> * allowed_contacts = nullptr) const
  {
    checkPatches(envelopeVertices(observed, delay, period), command, stage, allowed_contacts);
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
    RecoveryTimer metric_timer(RecoveryMetric::PROJECTION);
    double value = std::numeric_limits<double>::infinity();
    const Point local_normal = rotate(normal, -yaw);
    for (const auto & p : part) {value = std::min(value, local_normal.dot(p));}
    return value;
  }
  static void appendPlane(std::vector<SafetyPlane> & planes, SafetyPlane plane)
  {
    RecoveryTimer metric_timer(RecoveryMetric::APPEND);
    for (auto & old : planes) {
      if (active_recovery_profile) {++active_recovery_profile->plane_comparisons;}
      if ((old.normal - plane.normal).squaredNorm() < 1e-20 &&
        std::abs(old.preview_time - plane.preview_time) < 1e-14 &&
        (old.origin_offset - plane.origin_offset).squaredNorm() < 1e-24)
      {
        if (active_recovery_profile) {++active_recovery_profile->plane_duplicates;}
        old.offset = std::max(old.offset, plane.offset); return;
      }
    }
    planes.push_back(plane);
  }

  bool samplePatches(
    const std::vector<MotionVertex> & vertices,
    const Control & command,
    const Point & base,
    double dt,
    std::vector<SafetyPlane> & planes,
    const std::unordered_set<size_t> * initial_contacts = nullptr) const
  {
    return patches(vertices, command, [&](const Patch & patch, const std::vector<Point> & part) {
        // Recovery 刚脱离时仍可能处于 1mm 内；不能用普通求解器的额外数值缓冲
        // 拒绝已经满足真实安全余量且正在远离的起点。最终连续几何检查仍保留。
        const double margin = margin_ + patch.error + (initial_contacts ? 0.0 : 0.001);
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
            // Recovery：初始已经接触的障碍栅格, 暂时不生成分离平面
            if (initial_contacts)
            {
                size_t index = static_cast<size_t>(y) * field_->geometry.width + x;
                if (initial_contacts->count(index) > 0) return true;
            }
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

  void checkPatches(
    const std::vector<MotionVertex> & vertices,
    const Control & command,
    const std::string & stage,
    const std::unordered_set<size_t> * allowed_contacts = nullptr) const
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
              // Recovery：只允许初始已经接触的栅格
              size_t index = static_cast<size_t>(y) * field_->geometry.width + x;

              if (allowed_contacts && allowed_contacts->count(index) > 0)
              {
                  return true;
              }
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
    RecoveryTimer metric_timer(RecoveryMetric::VISIT);
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
      if (!unpaired_dynamic_) {
        // 同一不可变快照按需索引查询到的行，保留原来的 y/x 访问顺序。
        auto found = occupied_rows_.find(y);
        if (found == occupied_rows_.end()) {
          std::vector<int> occupied;
          const size_t offset = static_cast<size_t>(y) * field_->geometry.width;
          for (unsigned x = 0; x < field_->geometry.width; ++x) {
            if (field_->fused_obstacles.data[offset + x] != 0) {occupied.push_back(x);}
          }
          found = occupied_rows_.emplace(y, std::move(occupied)).first;
        }
        const auto & row = found->second;
        for (auto it = std::lower_bound(row.begin(), row.end(), x0);
          it != row.end() && *it <= x1; ++it)
        {
          if (active_recovery_profile) {++active_recovery_profile->obstacle_candidates;}
          if (!visitor(*it, y)) {return false;}
        }
      } else {
        // 尚未配对的最新动态层继续逐格查询原始谓词，不能用旧索引排除新障碍。
        for (int x = x0; x <= x1; ++x) {
          const auto raw = field_->fused_obstacles.data[static_cast<size_t>(y) * field_->geometry.width + x];
          if (raw != 0 || grid_.latestOccupied(x, y)) {
            if (active_recovery_profile) {++active_recovery_profile->obstacle_candidates;}
            if (!visitor(x, y)) {return false;}
          }
        }
      }
    }
    return true;
  }
  // 实例不跨地图/轮廓复用：地图版本、动态视图或轮廓变化自然建立新缓存。
  mutable std::unordered_map<int, std::vector<int>> occupied_rows_;
  mutable std::map<std::array<double, 3>, std::unordered_set<size_t>> collision_cache_;
  bool unpaired_dynamic_{false};
  map_server::MapView grid_;
  std::shared_ptr<const map_server::DerivedMaps> field_;
  std::vector<std::vector<Point>> parts_;
  double margin_, trust_region_, vertex_extent_{0};
};
// 两层独立复检共享相同的冻结规则；地图变化不能扩大初始接触权限。
struct RecoveryContacts
{
  std::unordered_set<size_t> cells;
  map_server::GridGeometry geometry;
  uint64_t producer{0};
  State previous;
  int count{0};
  bool ready{false};

  void initialize(const map_server::MapView & map, const TrajectorySafety & safety,
    const State & start)
  {
    *this = RecoveryContacts{};
    count = safety.collisionCellCount(start, &cells);
    if (count < 0) {throw std::runtime_error("Recovery 初始接触集合初始化失败");}
    geometry = map.geometry();
    producer = map.status().producer_id;
    previous = start;
    ready = true;
    if (!safety.checkEscapeMotion(start, start, &cells)) {
      ready = false;
      throw std::runtime_error("Recovery 初始轮廓检查失败");
    }
  }

  void update(const map_server::MapView & map, const TrajectorySafety & safety,
    const State & observed)
  {
    const auto & current_geometry = map.geometry();
    if (!ready || geometry.width != current_geometry.width ||
      geometry.height != current_geometry.height || geometry.resolution != current_geometry.resolution ||
      geometry.origin_x != current_geometry.origin_x || geometry.origin_y != current_geometry.origin_y ||
      producer != map.status().producer_id) {
      throw std::runtime_error("Recovery 初始集合未就绪、地图几何变化或生产端重启");
    }
    const int current = safety.collisionCellCount(observed);
    State from = previous;
    // TF 的 yaw 在 ±π 处折返，跨周期复检使用相邻的实际朝向。
    from.yaw = observed.yaw + std::atan2(std::sin(previous.yaw - observed.yaw),
      std::cos(previous.yaw - observed.yaw));
    if (current < 0 || current > count ||
      !safety.checkEscapeMotion(from, observed, &cells))
    {throw std::runtime_error("Recovery 实际接触增加、新接触或脱离后重新碰撞");}
    count = current;
    previous = observed;
    // 实际完全脱离后，本次运动永久恢复严格约束。
    if (count == 0) {cells.clear();}
  }
};
}  // namespace minco_controller
