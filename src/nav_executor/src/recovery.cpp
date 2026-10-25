#include "nav_executor/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav_executor
{
RecoveryConfig RecoveryConfig::load(const rclcpp::Node::SharedPtr & node)
{
  RecoveryConfig c;
  const auto parameter = [&](const std::string & name, auto & value) {
      value = node->declare_parameter("recovery." + name, value);
    };
  parameter("enabled", c.enabled);
  parameter("speed", c.speed);
  parameter("max_distance", c.max_distance);
  parameter("timeout", c.timeout);
  parameter("max_attempts", c.max_attempts);
  parameter("cooldown", c.cooldown);
  const auto get = [&](const std::string & key) {return node->get_parameter("MPC." + key).as_double();};
  c.collision_margin = get("collision_margin");
  c.delay = get("control_delay_compensation");
  double speed_limit = INFINITY, acceleration_limit = INFINITY;
  for (const auto & axis : {std::string("x"), std::string("y")}) {
    speed_limit = std::min({speed_limit, get("v" + axis + "_max"), -get("v" + axis + "_min")});
    acceleration_limit = std::min({acceleration_limit, get("a" + axis + "_max"), -get("a" + axis + "_min")});
  }
  if (!std::isfinite(speed_limit) || !std::isfinite(acceleration_limit) ||
    c.speed > speed_limit || acceleration_limit <= 0)
  {throw std::invalid_argument("Recovery 速度或 MPC 底盘运动限制无效");}
  // 低速动作使用保守加速度，保持与 MPC 同一物理上限；不增加一套底盘参数。
  c.acceleration = c.braking = std::min(acceleration_limit, 2 * c.speed);
  c.velocity_tolerance = node->get_parameter("stopped_velocity_tolerance").as_double() +
    c.acceleration * c.delay;
  c.planning_timeout = std::min(0.15, node->get_parameter("control_timeout").as_double() * 0.75);
  c.settle_time = std::max(0.3, c.delay + 1.0 / node->get_parameter("controller_frequency").as_double());
  c.stop_timeout = std::min(c.timeout, std::max(3.0, c.speed / c.braking + c.delay));
  c.total_distance = c.max_distance * c.max_attempts;
  c.progress_timeout = std::min(3.0, c.timeout / 3.0);
  c.validate();
  return c;
}

void RecoveryConfig::validate() const
{
  for (const auto value : {speed, max_distance, timeout, cooldown, total_distance,
    planning_timeout, stop_timeout, settle_time, acceleration, braking, velocity_tolerance,
    waypoint_tolerance, tracking_tolerance, progress_timeout, minimum_progress})
  {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("Recovery 速度、距离、时限及继承的底盘限制必须为正有限数");
    }
  }
  if (max_attempts < 1 || max_attempts > 20 || max_distance < 0.1 || max_distance > 5.0 ||
    !std::isfinite(collision_margin) || collision_margin < 0 ||
    !std::isfinite(delay) || delay < 0 || delay > 2.0)
  {throw std::invalid_argument("Recovery 尝试次数、距离或共用安全约束无效");}

}

RecoveryPlanner::RecoveryPlanner(RecoveryConfig config) : config_(config)
{
  config_.validate();
}

RecoveryPlan RecoveryPlanner::plan(const map_server::MapView & map,
  const map_server::Footprint & footprint, const minco_controller::State & start,
  const Eigen::Vector2d & reverse, const std::vector<Eigen::Vector2d> & previous_endpoints,
  Allowed allowed) const
{
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration<double>(config_.planning_timeout);
  const minco_controller::TrajectorySafety safety(map, footprint, config_.collision_margin);
  safety.checkPose(start, "Recovery 起始轮廓");
  RecoveryPlan result;
  result.producer = map.status().producer_id;
  result.version = map.version();
  result.dynamic_version = map.status().dynamic_version;
  const Eigen::Vector2d backward = reverse.norm() > 1e-6 ? reverse.normalized().eval() :
    Eigen::Vector2d::Zero().eval();
  std::vector<Eigen::Vector2d> directions;
  const auto addGradient = [&](double x, double y) {
      map_server::DistanceSample value;
      if (map.sampleDistance(x, y, value)) {
        const Eigen::Vector2d gradient(value.dx, value.dy);
        if (gradient.norm() > 1e-5) {directions.push_back(gradient.normalized());}
      }
    };
  addGradient(start.x, start.y);
  // 轮廓周围的梯度可以在中心梯度为零时给出开阔方向。
  for (const auto & vertex : map_server::transformFootprint(start.x, start.y, start.yaw, footprint)) {
    addGradient(vertex.x, vertex.y);
  }
  if (backward.norm() > 0) {directions.push_back(backward);}
  for (int i = 0; i < 16; ++i) {
    const double direction = 2 * M_PI * i / 16;
    directions.emplace_back(std::cos(direction), std::sin(direction));
  }
  const double start_clearance = safety.clearance(start, start, 0.25);
  const double step = std::clamp(map.geometry().resolution * 2, 0.05, 0.15);
  const double minimum_escape = std::min(0.3, config_.max_distance * 0.5);
  std::vector<Eigen::Vector2d> sampled;
  for (const auto & direction : directions) {
    if (std::any_of(sampled.begin(), sampled.end(), [&](const auto & d) {
      return d.dot(direction) > 0.999;
    })) {continue;}
    sampled.push_back(direction);
  }
  std::vector<bool> blocked(sampled.size(), false);
  // 先公平检查各方向的最短有效动作，再逐轮延长，避免一个长方向耗尽搜索预算。
  for (double length = minimum_escape; length <= config_.max_distance + 1e-9; length += step) {
    for (size_t i = 0; i < sampled.size(); ++i) {
      if (Clock::now() >= deadline) {break;}
      if (blocked[i]) {continue;}
      const auto & direction = sampled[i];
      auto end = start;
      end.x += direction.x() * length;
      end.y += direction.y() * length;
      if (allowed && !allowed(end)) {blocked[i] = true; continue;}
      try {safety.checkMotion(start, end, "Recovery 单方向连续扫掠");}
      catch (const std::runtime_error &) {blocked[i] = true; continue;}
      double integral_cost = 0;
      const int count = static_cast<int>(std::ceil(length / step));
      bool valid = true;
      for (int k = 1; k <= count; ++k) {
        const auto cell = map.cell(Eigen::Vector2d(start.x, start.y) + length * k / count * direction);
        if (!cell) {valid = false; break;}
        const double cost = map.derived().costmap.data.at(*cell);
        if (std::isnan(cost)) {valid = false; break;}
        // 无穷浮点代价属于圆形前端规划掩码，可行性仍由真实 Footprint 判断。
        integral_cost += length / count * (std::isfinite(cost) ? cost : 5.0);
      }
      if (!valid) {blocked[i] = true; continue;}
      bool repeated = false;
      for (const auto & old : previous_endpoints) {
        repeated = repeated || (Eigen::Vector2d(end.x, end.y) - old).norm() < minimum_escape;
      }
      if (repeated) {continue;}
      RecoveryCandidate candidate;
      candidate.states = {start, end};
      candidate.clearance = safety.clearance(start, end, 0.25);
      candidate.length = length;
      candidate.cost = integral_cost;
      const double clearance_gain = safety.clearance(end, end, 0.25) - start_clearance;
      // 直线平移没有换向项；优先净空和脱困进展，同时惩罚距离及累计浮点代价。
      candidate.score = 5.0 * candidate.clearance + 2.0 * (length + clearance_gain) +
        length * direction.dot(backward) - 0.4 * length - integral_cost;
      if (candidate.score > result.selected.score) {result.selected = candidate;}
      if (result.candidates.size() < 128) {result.candidates.push_back(std::move(candidate));}
    }
    if (Clock::now() >= deadline) {break;}
  }
  if (result.selected.states.empty()) {throw std::runtime_error("Recovery 无安全且有进展的轨迹，保持停车");}
  return result;
}

nav_msgs::msg::Path recoveryPath(const RecoveryCandidate & candidate, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  for (const auto & state : candidate.states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.z = std::sin(state.yaw / 2);
    pose.pose.orientation.w = std::cos(state.yaw / 2);
    path.poses.push_back(pose);
  }
  return path;
}
}  // namespace nav_executor
