#include "nav_executor/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav_executor
{

// ----------------------------结构体-----------------------------
RecoveryConfig RecoveryConfig::load(const rclcpp::Node::SharedPtr & node)
{
  RecoveryConfig c;
  const auto parameter = [&](const std::string & name, auto & value) {
      value = node->declare_parameter("recovery." + name, value);
    };

  // params.yaml中的参数
  parameter("enabled", c.enabled);
  parameter("speed", c.speed);
  parameter("max_distance", c.max_distance);
  parameter("timeout", c.timeout);
  parameter("max_attempts", c.max_attempts);
  parameter("cooldown", c.cooldown);

  // 复用 mpc 的 collision_margin 和 control_delay_compensation 两个参数
  // get() 读取 MPC 的参数
  const auto get = [&](const std::string & key) {return node->get_parameter("MPC." + key).as_double();};
  c.collision_margin = get("collision_margin");
  c.delay = get("control_delay_compensation");

  // 初始化 速度 和 加速度
  double speed_limit = INFINITY;
  double acceleration_limit = INFINITY;

  for (const auto & axis : {std::string("x"), std::string("y")}) {      // 两轮for循环，第一轮读vx_max ax_max
    speed_limit = std::min({speed_limit, get("v" + axis + "_max"), -get("v" + axis + "_min")});
    acceleration_limit = std::min({acceleration_limit, get("a" + axis + "_max"), -get("a" + axis + "_min")});
  }

  // 检查 速度 和 加速度 参数合法性
  if (!std::isfinite(speed_limit) || !std::isfinite(acceleration_limit) ||
    c.speed > speed_limit || acceleration_limit <= 0)
  {
    throw std::invalid_argument("Recovery 速度或 MPC 底盘运动限制无效");
  }

  // 低速动作使用保守加速度，保持与 MPC 同一物理上限；不增加一套底盘参数。
  c.acceleration = c.braking = std::min(acceleration_limit, 2 * c.speed);
  c.velocity_tolerance = node->get_parameter("stopped_velocity_tolerance").as_double() + c.acceleration * c.delay;
  c.planning_timeout = std::min(0.15, node->get_parameter("control_timeout").as_double() * 0.75);
  c.settle_time = std::max(0.3, c.delay + 1.0 / node->get_parameter("controller_frequency").as_double());
  c.stop_timeout = std::min(c.timeout, std::max(5.0, c.speed / c.braking + c.delay));
  c.total_distance = c.max_distance * c.max_attempts;
  c.progress_timeout = std::min(3.0, c.timeout / 3.0);
  c.validate();
  return c;
}

// 检查所有内部参数合法性
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
// ----------------------------结构体-----------------------------


// 类名::构造函数名
RecoveryPlanner::RecoveryPlanner(RecoveryConfig config) : config_(config)
{
  config_.validate();
}


// 返回类型 类名::plan
RecoveryPlan RecoveryPlanner::plan(
    const map_server::MapView & map,
    const map_server::Footprint & footprint,
    const minco_controller::State & start,
    const Eigen::Vector2d & reverse,
    const std::vector<Eigen::Vector2d> & previous_endpoints) const
{
    using Clock = std::chrono::steady_clock;

    // 设置搜索超时时间
    const auto deadline = Clock::now() + std::chrono::duration<double>(config_.planning_timeout);

    // 创建碰撞检测对象
    const minco_controller::TrajectorySafety safety(map, footprint, config_.collision_margin);

    // 起点碰撞检查
    std::unordered_set<size_t> initial_contacts;
    int start_collision = safety.collisionCellCount(start, &initial_contacts);
    if (start_collision < 0) {throw std::runtime_error("起点碰撞栅格数不合法，为-1");}

    if (start_collision == 0) {
        safety.checkPose(start, "Recovery起点碰撞检查");
    }

    // 初始化规划结果
    RecoveryPlan result;
    result.producer = map.status().producer_id;
    result.version = map.version();
    result.dynamic_version = map.status().dynamic_version;

    // 获取原导航路径的反方向
    const Eigen::Vector2d backward = reverse.norm() > 1e-6 ? reverse.normalized().eval() : Eigen::Vector2d::Zero().eval();
    // 生成候选搜索方向
    std::vector<Eigen::Vector2d> directions;
    // 添加 EDT 梯度方向
    const auto addGradient = [&](double x, double y) {
        map_server::DistanceSample value;

        if (map.sampleDistance(x, y, value)) {
            const Eigen::Vector2d gradient(value.dx, value.dy);

            if (gradient.allFinite() && gradient.norm() > 1e-5) {
                directions.push_back(gradient.normalized());
            }
        }
    };

    // 机器人中心的 EDT 梯度
    addGradient(start.x, start.y);

    // 机器人 Footprint 各顶点的 EDT 梯度
    for (const auto & vertex : map_server::transformFootprint(start.x, start.y, start.yaw, footprint))
    {
        addGradient(vertex.x, vertex.y);
    }

    // 加入原路径反方向
    if (backward.norm() > 0.0) {
        directions.push_back(backward);
    }

    // 36 个均匀分布的搜索方向
    constexpr int angle_samples = 36;
    for (int i = 0; i < angle_samples; ++i) {
        const double theta = 2.0 * M_PI * i / angle_samples;
        directions.emplace_back(std::cos(theta), std::sin(theta));
    }

    // 去除重复方向
    std::vector<Eigen::Vector2d> sampled;
    for (const auto & direction : directions) {
        const bool duplicated = std::any_of(sampled.begin(), sampled.end(), [&](const auto & d) {return d.dot(direction) > 0.999;});
        if (!duplicated) {
            sampled.push_back(direction);
        }
    }

    // 设置搜索距离
    const double step = std::clamp(map.geometry().resolution * 2.0, 0.05, 0.15);  // 搜索步长，现在是 0.1m
    const double minimum_escape = std::min(0.3, config_.max_distance * 0.5);

    // 终点需要额外 5cm 的净空
    // safety.clearance() 已扣除 collision_margin
    constexpr double min_goal_clearance = 0.05;

    // 记录已经被障碍阻断的方向
    std::vector<bool> blocked(sampled.size(), false);

    // 从近到远搜索
    for (double length = minimum_escape; length <= config_.max_distance + 1e-9; length += step)
    {
        // 当前搜索半径下的最佳候选
        RecoveryCandidate best_in_ring;

        // 遍历所有搜索方向
        for (size_t i = 0; i < sampled.size(); ++i)
        {
            if (Clock::now() >= deadline) {
                break;
            }

            if (blocked[i]) {
                continue;
            }

            const auto & direction = sampled[i];

            // 计算候选目标点
            auto end = start;
            end.x += direction.x() * length;
            end.y += direction.y() * length;

            // 检查起点到终点的连续运动是否安全
            if (start_collision > 0) {     // 如果已经发生碰撞
                if (!safety.checkEscapeMotion(start, end, &initial_contacts) ||
                    safety.collisionCellCount(end) != 0) {    // 撞了就搜下一个方向
                        continue;
                }
            } else {   // 没有发生碰撞
                try{
                    safety.checkMotion(start, end, "Recovery 路径是否有碰撞");
                }
                catch (const std::runtime_error &)
                {
                    blocked[i] = true;
                    continue;
                }
            }

            // 检查终点净空
            const double end_clearance = safety.clearance(end, end, 0.25);
            if (end_clearance < min_goal_clearance) {
                // 当前距离净空不够
                // 不阻断方向，允许下一轮继续向远处搜索
                continue;
            }

            // 避免重复选择之前尝试过的终点
            bool repeated = false;
            for (const auto & old : previous_endpoints) {
                const Eigen::Vector2d current_end(end.x, end.y);

                if ((current_end - old).norm() < minimum_escape) {
                    repeated = true;
                    break;
                }
            }

            if (repeated) {
                continue;
            }

            // 计算轨迹累计代价
            double integral_cost = 0.0;
            const int count = static_cast<int>(std::ceil(length / step));
            bool valid = true;

            for (int k = 1; k <= count; ++k) {
                const Eigen::Vector2d point = Eigen::Vector2d(start.x, start.y) + length * static_cast<double>(k) / count * direction;
                const auto cell = map.cell(point);

                if (!cell) {
                    valid = false;
                    break;
                }

                const double cost = map.derived().costmap.data.at(*cell);

                if (std::isnan(cost)) {
                    valid = false;
                    break;
                }

                // 延续现有的无穷代价处理方式
                integral_cost += length / count * (std::isfinite(cost) ? cost : 5.0);
            }

            if (!valid) {
                blocked[i] = true;
                continue;
            }

            // 构造候选轨迹
            RecoveryCandidate candidate;
            candidate.states = {start, end};
            candidate.length = length;
            candidate.clearance = safety.clearance(start, end, 0.25);
            candidate.cost = integral_cost;

            // 当前半径内的候选点评分
            candidate.score = 5.0 * end_clearance + 2.0 * candidate.clearance - integral_cost + 0.2 * direction.dot(backward);

            // 保存当前半径的最佳候选
            if (candidate.score > best_in_ring.score) {
                best_in_ring = candidate;
            }

            // 保存部分候选轨迹供 RViz 可视化
            if (result.candidates.size() < 128) {
                result.candidates.push_back(std::move(candidate));
            }
        }

        // 当前半径找到安全目标点，立即结束搜索
        if (!best_in_ring.states.empty()) {
            result.selected = std::move(best_in_ring);
            break;
        }

        // 超出搜索时间结束搜索
        if (Clock::now() >= deadline) {
            break;
        }
    }

    // 搜索失败
    if (result.selected.states.empty()) {
        throw std::runtime_error("Recovery 未找到满足终点净空要求的安全轨迹，保持停车");
    }

    // 返回规划结果
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
