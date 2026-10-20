// Adapted from Walker152/navi_minco_bit, commit 8288b7520ecb635bb27f16b0d49e3cf304f37a72.
#include "minco_controller/minco_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "map_server/costmap_client.hpp"
#include "map_server/footprint.hpp"
#include "minco_controller/costmap_cost.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_controller
{
namespace
{
double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
}
}

void MincoMpcController::initialize(const rclcpp::Node::WeakPtr & parent,
  const std::string & name, const std::shared_ptr<map_server::CostmapClient> & costmap)
{
  const auto node = parent.lock();
  if (solver_) {throw std::logic_error("MPC controller is already initialized");}
  if (!node || !costmap || name.empty()) {
    throw std::invalid_argument("MPC requires a node, instance name and CostmapClient");
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto parameter = [&](const std::string & key, auto value) {
      using T = decltype(value);
      const auto full_name = name + "." + key;
      if (!node->has_parameter(full_name)) {
        return node->declare_parameter<T>(full_name, value, descriptor);
      }
      return node->get_parameter(full_name).get_value<T>();
    };
  config_.dt = parameter("dt", 0.05);
  config_.horizon = parameter("horizon", 10);
  const auto q = parameter("Q", std::vector<double>{5.0, 5.0, 2.0});
  const auto r = parameter("R", std::vector<double>{1.0, 1.0, 0.5});
  if (q.size() != 3 || r.size() != 3) {throw std::invalid_argument("MPC Q and R require 3 weights");}
  config_.Q = Eigen::Vector3d(q[0], q[1], q[2]);
  config_.R = Eigen::Vector3d(r[0], r[1], r[2]);
  const auto delta_weight = parameter("control_delta_weight", std::vector<double>{2.0, 2.0, 0.5});
  if (delta_weight.size() != 3) {
    throw std::invalid_argument("MPC control_delta_weight requires 3 weights");
  }
  config_.control_delta_weight = Eigen::Vector3d(delta_weight[0], delta_weight[1], delta_weight[2]);
  config_.q_along = parameter("q_along", 3.0);
  config_.q_cross = parameter("q_cross", 12.0);
  config_.vx_min = parameter("vx_min", -1.0);
  config_.vx_max = parameter("vx_max", 1.0);
  config_.vy_min = parameter("vy_min", -1.0);
  config_.vy_max = parameter("vy_max", 1.0);
  config_.omega_min = parameter("omega_min", -2.0);
  config_.omega_max = parameter("omega_max", 2.0);
  config_.use_acc_constraints = parameter("use_acc_constraints", true);
  config_.ax_min = parameter("ax_min", -2.0);
  config_.ax_max = parameter("ax_max", 2.0);
  config_.ay_min = parameter("ay_min", -2.0);
  config_.ay_max = parameter("ay_max", 2.0);
  config_.alpha_min = parameter("alpha_min", -4.0);
  config_.alpha_max = parameter("alpha_max", 4.0);
  config_.max_working_set_recalculations = parameter("max_working_set_recalculations", 200);
  config_.eps_regularization = parameter("eps_regularization", 1e-9);
  config_.obstacle_weight = parameter("obstacle_weight", 30.0);
  config_.obstacle_trust_region = parameter("obstacle_trust_region", 0.15);
  config_.obstacle_iterations = parameter("obstacle_iterations", 3);
  obstacle_smoothing_distance_ = parameter("obstacle_smoothing_distance", 0.2);
  if (!std::isfinite(obstacle_smoothing_distance_) || obstacle_smoothing_distance_ <= 0.0 ||
    obstacle_smoothing_distance_ > 2.0)
  {throw std::invalid_argument("MPC obstacle_smoothing_distance must be in (0, 2] metres");}
  reference_speed_ = parameter("reference_speed", 0.6);
  reference_acceleration_ = parameter("reference_acceleration", 1.0);
  reference_yaw_rate_ = parameter("reference_yaw_rate", 1.0);
  control_delay_ = parameter("control_delay_compensation", 0.25);
  config_.safety_preview_time = control_delay_;
  const double frequency = node->has_parameter("controller_frequency") ?
    node->get_parameter("controller_frequency").as_double() : 20.0;
  if (!std::isfinite(frequency) || frequency <= 0.0) {
    throw std::invalid_argument("MPC controller_frequency must be positive");
  }
  config_.control_period = 1.0 / frequency;
  output_delay_ = parameter("output_delay", 0.025);
  deadzone_ = parameter("deadzone_speed_threshold", 0.02);
  velocity_noise_threshold_ = parameter("velocity_noise_threshold", 0.03);
  small_gyro_mode_ = parameter("use_small_gyro_mode", true);
  fixed_wz_ = parameter("fixed_wz", 0.0);
  collision_margin_ = parameter("collision_margin", 0.02);
  for (const auto value : {reference_speed_, reference_acceleration_, reference_yaw_rate_}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("MPC reference motion limits must be finite and positive");
    }
  }
  for (const auto value : {control_delay_, output_delay_, deadzone_, velocity_noise_threshold_,
    collision_margin_})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("MPC delays, deadzone, noise threshold and margin must be nonnegative");
    }
  }
  if (control_delay_ > 2.0 || output_delay_ > 1.0 || !std::isfinite(fixed_wz_) ||
    fixed_wz_ < config_.omega_min || fixed_wz_ > config_.omega_max)
  {throw std::invalid_argument("Invalid MPC delay or fixed angular velocity");}
  // A fixed-frame facade such as fake_baselink drops angular commands. Its
  // deployment uses the upstream small-gyro mode with fixed_wz=0.
  solver_ = std::make_unique<MpcSolver>(config_);
  node_ = parent;
  costmap_ = costmap;
  RCLCPP_DEBUG(node->get_logger(), "Configured MPC %s: dt=%.3f horizon=%d frame=%s base=%s",
    name.c_str(), config_.dt, config_.horizon, costmap->getGlobalFrameID().c_str(),
    costmap->getBaseFrameID().c_str());
}

void MincoMpcController::reset()
{
  if (solver_) {solver_->reset();}
  trajectory_.clear();
  plan_frame_.clear();
  progress_time_ = 0.0;
}

void MincoMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!solver_) {throw std::logic_error("Initialize MPC before setting a plan");}
  reset();
  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw std::invalid_argument("MPC requires a nonempty path and frame");
  }
  std::vector<ReferencePoint> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    if (!finitePose(pose.pose) || (!pose.header.frame_id.empty() &&
      pose.header.frame_id != path.header.frame_id))
    {throw std::invalid_argument("MPC path has invalid poses or inconsistent frames");}
    ReferencePoint point;
    point.pos = Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
    point.yaw = tf2::getYaw(pose.pose.orientation);
    points.push_back(point);
  }
  PathTrajectory candidate;
  candidate.setPath(points, reference_speed_, reference_acceleration_, reference_yaw_rate_,
    !small_gyro_mode_);
  trajectory_ = std::move(candidate);
  plan_frame_ = path.header.frame_id;
}

void MincoMpcController::checkPrediction(
  const std::vector<State> & prediction, const std::string & stage) const
{
  auto * grid = costmap_->getCostmap();
  std::unique_lock<map_server::Costmap2D::mutex_t> lock(*grid->getMutex());
  const double resolution = grid->getResolution();
  if (grid->getSizeInCellsX() < 3 || grid->getSizeInCellsY() < 3 || resolution <= 0.0) {
    throw std::runtime_error("MPC local costmap has no usable grid");
  }
  const auto & footprint = costmap_->getRobotFootprint();
  const double radius = (footprint.empty() ? costmap_->settings().robot_radius :
    map_server::calculateMinAndMaxDistances(footprint).second) + collision_margin_;
  // The 253 band already accounts for the robot's inscribed radius: check it
  // at the center only. Sweep the full disc against lethal/unknown cells so
  // that inflation does not count the robot radius a second time.
  const auto check = [&](double x, double y, size_t step) {
      unsigned int mx, my;
      if (!grid->worldToMap(x - radius, y - radius, mx, my) ||
        !grid->worldToMap(x + radius, y + radius, mx, my))
      {throw std::runtime_error("MPC " + stage + " leaves the local costmap");}
      const double reach = radius + resolution * std::sqrt(0.5);
      const int cells = static_cast<int>(std::ceil(reach / resolution));
      grid->worldToMap(x, y, mx, my);
      // const auto center_cost = grid->getCost(mx, my);
      // if (center_cost >= map_server::INSCRIBED_INFLATED_OBSTACLE) {
      //   throw std::runtime_error("MPC prediction center is inscribed, lethal or unknown: x=" +
      //           std::to_string(x) + " y=" + std::to_string(y) +
      //           " cost=" + std::to_string(center_cost));
      // }
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int ix = static_cast<int>(mx) + dx, iy = static_cast<int>(my) + dy;
          if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid->getSizeInCellsX()) ||
            iy >= static_cast<int>(grid->getSizeInCellsY())) {continue;}
          double wx, wy;
          grid->mapToWorld(ix, iy, wx, wy);
          if (std::hypot(wx - x, wy - y) > reach) {continue;}

          const auto cost = grid->getCost(ix, iy);
          // 只判断是否碰撞真实障碍
          if (cost != map_server::LETHAL_OBSTACLE) {
            continue;
          }
          // 栅格尺寸为 resolution × resolution
          const double half = resolution * 0.5;

          // 计算圆心到方形栅格的最近距离
          const double distance_x = std::max(std::abs(wx - x) - half, 0.0);
          const double distance_y = std::max(std::abs(wy - y) - half, 0.0);

          // 圆和栅格方块的精确几何相交检测
          const double distance_sq = distance_x * distance_x + distance_y * distance_y;
          const double radius_sq = radius * radius;

          if (distance_sq <= radius_sq) {
            throw std::runtime_error(
              "MPC " + stage + " footprint collision: "
              "center_x=" + std::to_string(x) +
              " center_y=" + std::to_string(y) +
              " cell_x=" + std::to_string(wx) +
              " cell_y=" + std::to_string(wy) +
              " distance=" + std::to_string(std::sqrt(distance_sq)) +
              " radius=" + std::to_string(radius) +
              " cost=" + std::to_string(cost) +
              " step=" + std::to_string(step));
          }
        }
      }
    };
  for (size_t i = 0; i < prediction.size(); ++i) {
    const auto & b = prediction[i];
    const auto & a = prediction[i ? i - 1 : 0];
    check(b.x, b.y, i);
    const int steps = std::max(1, static_cast<int>(std::ceil(
          std::hypot(b.x - a.x, b.y - a.y) / (0.5 * resolution))));
    for (int k = 0; k <= steps; ++k) {
      const double fraction = static_cast<double>(k) / steps;
      check(a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y), i);
    }
  }
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured_speed)
{
  const auto node = node_.lock();
  if (!node || !solver_ || trajectory_.empty()) {throw std::logic_error("MPC has no active plan");}
  if (pose.header.frame_id != costmap_->getGlobalFrameID() || !finitePose(pose.pose) ||
    !Eigen::Vector3d(measured_speed.linear.x, measured_speed.linear.y,
      measured_speed.angular.z).allFinite())
  {throw std::invalid_argument("MPC received an invalid pose, frame or measured velocity");}
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  // PathExecutor already transformed the odometry child frame and lever arm
  // into the configured base frame. Rotate once to the solver's global frame.
  State current;
  current.x = pose.pose.position.x;
  current.y = pose.pose.position.y;
  current.yaw = yaw;
  current.vx = c * measured_speed.linear.x - s * measured_speed.linear.y;
  current.vy = s * measured_speed.linear.x + c * measured_speed.linear.y;
  current.omega = measured_speed.angular.z;
  const auto last_command = solver_->getLastControl();
  int axis = 0;
  for (auto * value : {&current.vx, &current.vy, &current.omega}) {
    // Suppress stationary odometry noise, but keep legitimate low-speed motion
    // once commanded. Otherwise a=1.5, dt=0.02 stalls at the 0.03 noise threshold.
    if (std::abs(*value) < velocity_noise_threshold_ &&
      std::abs(last_command[axis]) < deadzone_) {*value = 0.0;}
    ++axis;
  }
  const State observed = current;
  // Preserve upstream forward state prediction and future-reference sampling.
  if (control_delay_ > 1e-3) {
    current.x += current.vx * control_delay_;
    current.y += current.vy * control_delay_;
    current.yaw = wrap(current.yaw + current.omega * control_delay_);
  }
  auto transform = geometry_msgs::msg::TransformStamped{};
  transform.transform.rotation.w = 1.0;
  if (plan_frame_ != pose.header.frame_id) {
    transform = costmap_->getTfBuffer()->lookupTransform(
      pose.header.frame_id, plan_frame_, tf2::TimePointZero);
  }
  if (std::abs(transform.transform.rotation.x) > 1e-3 ||
    std::abs(transform.transform.rotation.y) > 1e-3)
  {throw std::runtime_error("MPC requires a planar transform between path and costmap frames");}
  const double offset = tf2::getYaw(transform.transform.rotation);
  const double ct = std::cos(offset), st = std::sin(offset);
  const double tx = transform.transform.translation.x, ty = transform.transform.translation.y;
  State in_plan = observed;
  in_plan.x = ct * (observed.x - tx) + st * (observed.y - ty);
  in_plan.y = -st * (observed.x - tx) + ct * (observed.y - ty);
  in_plan.yaw = observed.yaw - offset;
  progress_time_ = trajectory_.nearestTime(in_plan, progress_time_);
  std::vector<ReferencePoint> reference;
  reference.reserve(config_.horizon);
  double previous_yaw = current.yaw;
  for (int k = 0; k < config_.horizon; ++k) {
    auto point = trajectory_.sample(progress_time_ + control_delay_ + k * config_.dt);
    const auto position = point.pos.eval();
    const auto velocity = point.vel.eval();
    point.pos = Eigen::Vector2d(ct * position.x() - st * position.y() + tx,
      st * position.x() + ct * position.y() + ty);
    point.vel = Eigen::Vector2d(ct * velocity.x() - st * velocity.y(),
      st * velocity.x() + ct * velocity.y());
    point.yaw = small_gyro_mode_ ? current.yaw :
      previous_yaw + wrap(point.yaw + offset - previous_yaw);
    if (small_gyro_mode_) {point.yaw_rate = fixed_wz_;}
    previous_yaw = point.yaw;
    reference.push_back(point);
  }
  Control control;
  std::vector<State> prediction;
  // Snapshot construction locks the grid, then releases it before optimization.
  // Positions, gradients and controls all use the local Costmap global frame.
  const auto & footprint = costmap_->getRobotFootprint();
  const double radius = (footprint.empty() ? costmap_->settings().robot_radius :
    map_server::calculateMinAndMaxDistances(footprint).second) + collision_margin_;
  const CostmapCost field(*costmap_->getCostmap(), obstacle_smoothing_distance_,
    radius, config_.obstacle_trust_region);
  // This segment is fixed by measured velocity and cannot be repaired by U.
  checkPrediction({observed, current}, "delay_projection");
  const auto sample_cost = [&field](const Eigen::Vector2d & p, CostLinearization & cost) {
      return field.sample(p, cost);
    };
  if (!solver_->solve(current, reference, control, &prediction, sample_cost, &observed)) {
    throw std::runtime_error("MPC qpOASES solve failed, status=" + std::to_string(solver_->lastStatus()));
  }
  // Upstream output phase compensation anticipates physical chassis rotation;
  // it is separate from the global-to-base coordinate conversion below.
  const double phase = current.omega * output_delay_;
  double vx = std::cos(phase) * control.vx + std::sin(phase) * control.vy;
  double vy = -std::sin(phase) * control.vx + std::cos(phase) * control.vy;
  double wz = small_gyro_mode_ ? fixed_wz_ : control.omega;
  vx = std::clamp(vx, config_.vx_min, config_.vx_max);
  vy = std::clamp(vy, config_.vy_min, config_.vy_max);
  if (config_.use_acc_constraints) {
    // Keep the phase correction inside the same first-step acceleration bounds.
    vx = std::clamp(vx, current.vx + config_.ax_min * config_.control_period,
      current.vx + config_.ax_max * config_.control_period);
    vy = std::clamp(vy, current.vy + config_.ay_min * config_.control_period,
      current.vy + config_.ay_max * config_.control_period);
  }
  if (std::hypot(vx, vy) < deadzone_) {vx = vy = 0.0;}
  prediction.insert(prediction.begin(), observed);
  checkPrediction(prediction, "qp_horizon");
  // Check the actual phase-compensated first command as well as the QP horizon.
  // Match the QP: measured velocity applies before actuation; the new command
  // applies after it. Do not apply the new command retroactively during latency.
  State next = current;
  next.x += vx * config_.dt;
  next.y += vy * config_.dt;
  checkPrediction({current, next}, "output_command");
  State held = observed;
  held.x += vx * (config_.control_period + control_delay_);
  held.y += vy * (config_.control_period + control_delay_);
  checkPrediction({observed, held}, "command_hold");
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node->now();
  command.header.frame_id = costmap_->getBaseFrameID();
  command.twist.linear.x = c * vx + s * vy;
  command.twist.linear.y = -s * vx + c * vy;
  command.twist.angular.z = wz;
  return command;  // Only PathExecutor publishes the command.
}
}  // namespace minco_controller
