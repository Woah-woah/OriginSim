#include "nav_executor/connector_executor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav_executor
{
namespace
{
using Point = geometry_msgs::msg::Point;
constexpr double EPS = 1e-9;
double cross(const Point & a, const Point & b, const Point & c)
{return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);}
bool onSegment(const Point & p, const Point & a, const Point & b)
{
  return std::abs(cross(a, b, p)) <= EPS &&
         p.x >= std::min(a.x, b.x)-EPS && p.x <= std::max(a.x, b.x)+EPS &&
         p.y >= std::min(a.y, b.y)-EPS && p.y <= std::max(a.y, b.y)+EPS;
}
bool edgesIntersect(const Point & a, const Point & b, const Point & c, const Point & d)
{
  const double ab_c = cross(a, b, c), ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a), cd_b = cross(c, d, b);
  return (((ab_c > EPS && ab_d < -EPS) || (ab_c < -EPS && ab_d > EPS)) &&
          ((cd_a > EPS && cd_b < -EPS) || (cd_a < -EPS && cd_b > EPS))) ||
         onSegment(a, c, d) || onSegment(b, c, d) ||
         onSegment(c, a, b) || onSegment(d, a, b);
}
bool inside(const Point & p, const std::vector<Point> & polygon)
{
  bool result = false;
  for (size_t i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
    const auto & a = polygon[j];
    const auto & b = polygon[i];
    if (onSegment(p, a, b)) {return true;}
    if ((a.y > p.y) != (b.y > p.y) &&
      p.x < (b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) {result = !result;}
  }
  return result;
}
Point point(double x, double y)
{Point result; result.x = x; result.y = y; return result;}
}  // namespace

ConnectorExecutor::ConnectorExecutor(ConnectorConfig config, std::string frame,
  std::vector<Point> footprint, Command command, Log log)
: config_(config), frame_(std::move(frame)), footprint_(std::move(footprint)),
  command_(std::move(command)), log_(std::move(log))
{
  for (double value : {config.region_half_width, config.pose_timeout}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Connector region width and pose timeout must be positive");
    }
  }
  for (double value : {config.entry_extension, config.exit_extension,
    config.exit_hysteresis, config.exit_debounce})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("Connector region margins and debounce must be nonnegative");
    }
  }
  if (frame_.empty() || !command_ || footprint_.size() < 3) {
    throw std::invalid_argument("Connector geometry requires frame, command and polygon footprint");
  }
  double area = 0.0;
  for (size_t i = 0; i < footprint_.size(); ++i) {
    const auto & a = footprint_[i];
    const auto & b = footprint_[(i+1)%footprint_.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y)) {
      throw std::invalid_argument("Connector footprint contains nonfinite coordinates");
    }
    area += a.x*b.y - b.x*a.y;
  }
  if (std::abs(area) <= EPS) {throw std::invalid_argument("Connector footprint has zero area");}
}

void ConnectorExecutor::setPlan(const topology_global_planner::TopologyPlan & plan)
{
  std::vector<Region> incoming;
  for (const auto & segment : plan.segments) {
    if (segment.type != topology_global_planner::SegmentType::CONNECTOR || segment.action != "down") {
      continue;
    }
    const auto & a = segment.wait_pose.pose.position;
    const auto & b = segment.exit_pose.pose.position;
    const double length = std::hypot(b.x-a.x, b.y-a.y);
    if (segment.connector_id.empty() || segment.wait_pose.header.frame_id != frame_ ||
      segment.exit_pose.header.frame_id != frame_ || !std::isfinite(a.x) || !std::isfinite(a.y) ||
      !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(length) || length <= EPS)
    {throw std::invalid_argument("Invalid down connector geometry: " + segment.connector_id);}
    Region region{segment.connector_id, a.x, a.y,
      (b.x-a.x)/length, (b.y-a.y)/length, length, {}, {}, true, false, std::nullopt};
    for (const double padding : {0.0, config_.exit_hysteresis}) {
      const double left = -config_.entry_extension-padding;
      const double right = length+config_.exit_extension+padding;
      const double width = config_.region_half_width+padding;
      auto polygon = std::vector<Point>{point(left, -width), point(right, -width),
        point(right, width), point(left, width)};
      if (region.polygon.empty()) {region.polygon = std::move(polygon);}
      else {region.exit_polygon = std::move(polygon);}
    }
    incoming.push_back(std::move(region));
  }
  // Validate the entire incoming plan before changing retained geometry.
  for (auto & region : regions_) {region.current = false;}
  for (const auto & candidate : incoming) {
    const auto existing = std::find_if(regions_.begin(), regions_.end(), [&](const Region & r) {
      return r.id == candidate.id && r.x == candidate.x && r.y == candidate.y &&
             r.ux == candidate.ux && r.uy == candidate.uy && r.length == candidate.length;
    });
    if (existing != regions_.end()) {existing->current = true;}
    else {regions_.push_back(candidate);}
  }
}

bool ConnectorExecutor::intersects(const Region & region,
  const std::vector<Point> & footprint, double padding) const
{
  const auto & rectangle = padding > 0.0 ? region.exit_polygon : region.polygon;
  const double left = rectangle[0].x, right = rectangle[1].x, width = rectangle[2].y;
  std::vector<Point> local;
  local.reserve(footprint.size());
  for (const auto & p : footprint) {
    const double dx = p.x-region.x, dy = p.y-region.y;
    local.push_back(point(dx*region.ux+dy*region.uy, -dx*region.uy+dy*region.ux));
    if (local.back().x >= left-EPS && local.back().x <= right+EPS &&
      std::abs(local.back().y) <= width+EPS) {return true;}
  }
  if (inside(rectangle.front(), local)) {return true;}
  // Edge intersections also cover crossed polygons with no contained vertices.
  for (size_t i = 0; i < local.size(); ++i) {
    for (size_t j = 0; j < rectangle.size(); ++j) {
      if (edgesIntersect(local[i], local[(i+1)%local.size()],
        rectangle[j], rectangle[(j+1)%rectangle.size()])) {return true;}
    }
  }
  return false;
}

void ConnectorExecutor::localizationLost()
{
  last_pose_at_.reset();
  for (auto & region : regions_) {region.outside_since.reset();}
}

void ConnectorExecutor::observePose(const geometry_msgs::msg::PoseStamped & pose, Clock::time_point now)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (pose.header.frame_id != frame_ || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(norm) || std::abs(norm-1.0) > 0.01)
  {localizationLost(); return;}
  if (last_pose_at_ && (now < *last_pose_at_ ||
    std::chrono::duration<double>(now-*last_pose_at_).count() > config_.pose_timeout))
  {localizationLost();}
  last_pose_at_ = now;
  // Project the actual 3D body orientation onto the ground plane (yaw plus tilt).
  const double r00 = 1.0-2.0*(q.y*q.y+q.z*q.z), r01 = 2.0*(q.x*q.y-q.z*q.w);
  const double r10 = 2.0*(q.x*q.y+q.z*q.w), r11 = 1.0-2.0*(q.x*q.x+q.z*q.z);
  std::vector<Point> world;
  world.reserve(footprint_.size());
  for (const auto & vertex : footprint_) {
    world.push_back(point(p.x+r00*vertex.x+r01*vertex.y, p.y+r10*vertex.x+r11*vertex.y));
  }
  bool lowered = false;
  for (auto & region : regions_) {
    if (intersects(region, world, region.occupied ? config_.exit_hysteresis : 0.0)) {
      region.outside_since.reset();
      if (!region.occupied && log_) {log_("Enter connector low-head region: " + region.id);}
      region.occupied = true;
    } else {
      if (!region.outside_since) {region.outside_since = now;}
      if (std::chrono::duration<double>(now-*region.outside_since).count() >= config_.exit_debounce) {
        if (region.occupied && log_) {log_("Leave connector low-head region: " + region.id);}
        region.occupied = false;
      }
    }
    lowered = lowered || region.occupied;
  }
  if (lowered != lowered_) {
    command_(lowered);
    lowered_ = lowered;
  }
  regions_.erase(std::remove_if(regions_.begin(), regions_.end(), [&](const Region & r) {
    return !r.current && !r.occupied && r.outside_since &&
           std::chrono::duration<double>(now-*r.outside_since).count() >= config_.exit_debounce;
  }), regions_.end());
}
}  // namespace nav_executor
