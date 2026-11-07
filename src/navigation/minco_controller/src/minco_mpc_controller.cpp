// Adapted from Walker152/navi_minco_bit, commit 8288b7520ecb635bb27f16b0d49e3cf304f37a72.
#include "minco_controller/minco_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

#include "map_server/map_client.hpp"
#include "map_server/footprint.hpp"
#include "minco_controller/costmap_cost.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_controller
{
namespace
{

double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 判断位姿合法性
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
}

}


void MincoMpcController::initialize(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & name,
  const std::shared_ptr<map_server::MapClient> & costmap)
{
  const auto node = parent.lock();
  if (solver_) {throw std::logic_error("局部控制器已经初始化，不能重复初始化");}
  if (!node || !costmap || name.empty()) {
    throw std::invalid_argument("局部控制器缺少节点、名称或地图客户端");
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

  if (q.size() != 3 || r.size() != 3) {throw std::invalid_argument("局部控制器的 Q、R 参数必须各包含三个权重");}
  config_.Q = Eigen::Vector3d(q[0], q[1], q[2]);
  config_.R = Eigen::Vector3d(r[0], r[1], r[2]);
  const auto delta_weight = parameter("control_delta_weight", std::vector<double>{2.0, 2.0, 0.5});

  if (delta_weight.size() != 3) {
    throw std::invalid_argument("速度变化权重必须包含三个数值");
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
  reference_speed_ = parameter("reference_speed", 0.6);
  reference_acceleration_ = parameter("reference_acceleration", 1.0);
  reference_yaw_rate_ = parameter("reference_yaw_rate", 1.0);
  control_delay_ = parameter("control_delay_compensation", 0.10);
  config_.safety_preview_time = control_delay_;

  const double frequency = node->get_parameter("controller_frequency").as_double();

  if (!std::isfinite(frequency) || frequency <= 0.0) {
    throw std::invalid_argument("控制频率必须大于零");
  }

  config_.control_period = 1.0 / frequency;
  output_delay_ = parameter("output_delay", 0.025);
  deadzone_ = parameter("deadzone_speed_threshold", 0.02);
  velocity_noise_threshold_ = parameter("velocity_noise_threshold", 0.03);
  small_gyro_mode_ = parameter("use_small_gyro_mode", true);
  fixed_wz_ = parameter("fixed_wz", 0.0);
  collision_margin_ = parameter("collision_margin", 0.02);


  for (const auto value : {reference_speed_, reference_acceleration_, reference_yaw_rate_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("参考速度、加速度和角速度上限必须是大于零的有限数值");
    }
  }


  for (const auto value : {control_delay_, output_delay_, deadzone_, velocity_noise_threshold_, collision_margin_})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("延迟、速度死区、噪声阈值和安全余量不能为负数或无效数值");
    }
  }


  if (control_delay_ > 2.0 || output_delay_ > 1.0 || !std::isfinite(fixed_wz_) ||
    fixed_wz_ < config_.omega_min || fixed_wz_ > config_.omega_max)
  {throw std::invalid_argument("控制延迟或固定角速度参数无效");}
  // 控制坐标系可保持固定朝向，真实轮廓通过同一观测时刻的 TF 转入该坐标系。
  solver_ = std::make_unique<MpcSolver>(config_);
  node_ = parent;
  costmap_ = costmap;
  RCLCPP_INFO(node->get_logger(),
    "局部控制时序：名义延迟=%.3f秒，安全检查覆盖的延迟范围=[0,%.3f]秒，指令保持周期=%.3f秒；"
    "求解器输出即最终全局坐标系速度",
    control_delay_, control_delay_, config_.control_period);
  RCLCPP_DEBUG(node->get_logger(), "局部控制器 %s 已配置：预测步长=%.3f秒，预测步数=%d，地图坐标系=%s，车体坐标系=%s",
    name.c_str(), config_.dt, config_.horizon, costmap->getGlobalFrameID().c_str(),
    costmap->getBaseFrameID().c_str());
}


void MincoMpcController::reset()
{
  if (solver_) {solver_->setConfig(config_);}
  trajectory_.clear();
  plan_frame_.clear();
  progress_time_ = 0.0;
  recovery_mode_ = false;
  recovery_contacts_ = {};
  recovery_acceleration_ = 0.0;
}


void MincoMpcController::notifyZeroCommandPublished()
{
  if (solver_) {solver_->setPreviousControl(Control{});}
}


void MincoMpcController::setPlan(
  const nav_msgs::msg::Path & path,
  const double recovery_speed,
  const double recovery_acceleration)
{
  if (!solver_) {
    throw std::logic_error("请先初始化局部控制器，再设置路径");
  }
  reset();

  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw std::invalid_argument("局部控制器收到空路径或缺少坐标系名称");
  }

  std::vector<ReferencePoint> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    if (!finitePose(pose.pose) || (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
    {
      throw std::invalid_argument("局部路径包含无效位姿或混用了不同坐标系");
    }

    ReferencePoint point;
    point.pos = Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
    point.heading_yaw = tf2::getYaw(pose.pose.orientation);
    points.push_back(point);
  }
  PathTrajectory candidate;
  // 是否正在设置 Recovery 路径
  const bool recovery = recovery_speed != 0.0 || recovery_acceleration != 0.0;

  if (recovery &&
      (!std::isfinite(recovery_speed) ||
      !std::isfinite(recovery_acceleration) ||
      recovery_speed <= 0.0 ||
      recovery_acceleration <= 0.0))
  {
      throw std::invalid_argument("Recovery 运动参数无效");
  }

  // Recovery 采用固定车头朝向
  if (recovery && (!small_gyro_mode_ || std::abs(fixed_wz_) > 1e-6))
  {
      throw std::invalid_argument("Recovery MPC 需要固定零角速度模式");
  }

  // 使用 Recovery 的参考速度和加速度
  candidate.setPath(
      points,
      recovery ? recovery_speed : reference_speed_,
      recovery ? recovery_acceleration : reference_acceleration_,
      reference_yaw_rate_,
      !small_gyro_mode_);

  // 复制正常 MPC 的约束
  MPCConfig active_config = config_;

  if (recovery)
  {
      // 保守限制 X、Y 两轴，确保平移合速度不超过 recovery_speed
      double v = recovery_speed / std::sqrt(2.0);
      double a = recovery_acceleration / std::sqrt(2.0);

      active_config.vx_max = std::min(active_config.vx_max, v);
      active_config.vx_min = std::max(active_config.vx_min, -v);

      active_config.vy_max = std::min(active_config.vy_max, v);
      active_config.vy_min = std::max(active_config.vy_min, -v);

      active_config.use_acc_constraints = true;

      active_config.ax_max = std::min(active_config.ax_max, a);
      active_config.ax_min = std::max(active_config.ax_min, -a);

      active_config.ay_max = std::min(active_config.ay_max, a);
      active_config.ay_min = std::max(active_config.ay_min, -a);
  }

  // 更新求解器限制
  solver_->setConfig(active_config);

  recovery_mode_ = recovery;
  recovery_acceleration_ = recovery ? recovery_acceleration : 0.0;

  trajectory_ = std::move(candidate);
  plan_frame_ = path.header.frame_id;
}

void MincoMpcController::setRecoveryContacts(const RecoveryContacts & contacts)
{
  if (!recovery_mode_ || !contacts.ready || recovery_contacts_.ready) {
    throw std::logic_error("Recovery 初始接触状态无效或重复初始化");
  }
  recovery_contacts_ = contacts;
}

void MincoMpcController::checkPrediction(
  const std::vector<State> & prediction, const std::string & stage,
  const TrajectorySafety & field) const
{
  RecoveryTimer timer(RecoveryMetric::PREDICTION);
  if (prediction.empty()) {throw ControllerInfeasible("MPC 预测轨迹为空");}
  try {
    for (size_t i = 0; i < prediction.size(); ++i) {
      const auto & previous = prediction[i ? i - 1 : 0];
      if (recovery_mode_) {
        if (!field.checkEscapeMotion(previous, prediction[i], &recovery_contacts_.cells)) {
          throw std::runtime_error(stage + "：Recovery 接触恶化或产生新接触");
        }
      } else {
        field.checkMotion(previous, prediction[i], stage);
      }
    }
  } catch (const std::runtime_error & error) {throw ControllerInfeasible(error.what());}
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured_speed)
{
  RecoveryTimer mpc_timer(RecoveryMetric::MPC);
  recoveryStage("mpc_setup");
  const auto node = node_.lock();
  if (!node || !solver_ || trajectory_.empty()) {throw std::logic_error("局部控制器尚未就绪，或没有可执行路径");}
  if (pose.header.frame_id != costmap_->getGlobalFrameID() || !finitePose(pose.pose) ||
    !Eigen::Vector3d(measured_speed.linear.x, measured_speed.linear.y,
      measured_speed.angular.z).allFinite())
  {throw std::invalid_argument("局部控制器收到无效位姿、坐标系或实测速度");}
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
    if (!recovery_mode_ && std::abs(*value) < velocity_noise_threshold_ &&
      std::abs(last_command[axis]) < deadzone_) {*value = 0.0;}
    ++axis;
  }
  const State observed = current;
  // 名义执行延迟取配置值；实际延迟未标定时，另以 [0, delay] 安全包络保护。
  if (control_delay_ > 0.0) {
    current.x += current.vx * control_delay_;
    current.y += current.vy * control_delay_;
    current.yaw += current.omega * control_delay_;  // 保留连续转角，避免跨 ±π 时误判转动方向。
  }
  auto transform = geometry_msgs::msg::TransformStamped{};
  transform.transform.rotation.w = 1.0;
  if (plan_frame_ != pose.header.frame_id) {
    transform = costmap_->getTfBuffer()->lookupTransform(
      pose.header.frame_id, plan_frame_, tf2::TimePointZero);
  }
  if (std::abs(transform.transform.rotation.x) > 1e-3 ||
    std::abs(transform.transform.rotation.y) > 1e-3)
  {throw std::runtime_error("路径与局部地图之间必须使用平面坐标变换");}
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
    auto point = trajectory_.sample(progress_time_ + control_delay_ + (k + 1) * config_.dt);
    const auto position = point.pos.eval();
    const auto velocity = point.vel.eval();
    point.pos = Eigen::Vector2d(ct * position.x() - st * position.y() + tx,
      st * position.x() + ct * position.y() + ty);
    point.vel = Eigen::Vector2d(ct * velocity.x() - st * velocity.y(),
      st * velocity.x() + ct * velocity.y());
    point.path_tangent_yaw = wrap(point.path_tangent_yaw + offset);
    point.heading_yaw = small_gyro_mode_ ? current.yaw + (k + 1) * config_.dt * fixed_wz_ :
      previous_yaw + wrap(point.heading_yaw + offset - previous_yaw);
    if (small_gyro_mode_) {point.yaw_rate = fixed_wz_;}
    previous_yaw = point.heading_yaw;
    reference.push_back(point);
  }
  Control control;
  std::vector<State> prediction;
  // 只复制不可变视图指针；距离、原始障碍与硬安全检查全程使用同一个地图版本。
  const auto footprint = costmap_->footprintInBaseFrame(pose.header.stamp);
  const auto captured = costmap_->view();
  if (!costmap_->isCurrent(*captured)) {
    if (costmap_->isUpdating()) {throw MapChanged("地图新版本尚在配对，当前周期停车等待");}
    throw std::runtime_error("局部地图快照无效或过期");
  }
  const auto & snapshot = *captured;
  const CostmapCost field(snapshot, footprint, collision_margin_, config_.obstacle_trust_region);

  if (recovery_mode_) {
    // 初始集合来自执行器停稳时的快照；求解期间绝不重新收集或扩大。
    recovery_contacts_.update(snapshot, field, observed);
  }
  const auto * escape_contacts = recovery_mode_ ? &recovery_contacts_.cells : nullptr;
  if (!recovery_mode_ || recovery_contacts_.count == 0) {
    field.checkPose(observed, "当前实际车体轮廓");
  }
  // This segment is fixed by measured velocity and cannot be repaired by U.
  try {
    recoveryStage("delay_safety");
    checkPrediction({observed, current}, "延迟期间按当前实测速度预测", field);
  } catch (const std::exception & trigger) {
    // 诊断与停车检查统一使用同一快照、同一 Footprint 和同一安全余量。
    try {
      const auto diagnose = [&](const State & state) {
          try {field.checkPose(state, "轮廓位姿检查"); return std::string("安全");}
          catch (const std::exception & error) {return std::string(error.what());}
        };
      const auto paired = snapshot.derivedMaps();
      const bool generated = costmap_->get_parameter("footprint").as_string().empty();
      RCLCPP_WARN(node->get_logger(),
        "延迟预测轮廓诊断：地图版本=%llu，时间戳=%lld纳秒，坐标系=%s，延迟=%.3f秒；"
        "实测速度=(%.4f,%.4f)米/秒，角速度=%.4f弧度/秒，轮廓来源=%s，顶点数=%zu，安全余量=%.4f米；"
        "整个移动过程检查未通过，原因=%s",
        static_cast<unsigned long long>(paired->version), static_cast<long long>(static_cast<int64_t>(snapshot.status().stamp_s * 1e9)),
        pose.header.frame_id.c_str(), control_delay_, observed.vx, observed.vy, observed.omega,
        generated ? "按半径生成的近似轮廓（含已有外扩）" : "配置的车体轮廓（含已有外扩）",
        footprint.size(), collision_margin_, trigger.what());
      RCLCPP_WARN(node->get_logger(), "当前实际位姿＋车体轮廓：%s；位姿=(%.6f,%.6f,%.6f)",
        diagnose(observed).c_str(), observed.x, observed.y, observed.yaw);
      RCLCPP_WARN(node->get_logger(), "延迟预测位姿＋车体轮廓：%s；位姿=(%.6f,%.6f,%.6f)",
        diagnose(current).c_str(), current.x, current.y, current.yaw);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node->get_logger(), "轮廓诊断未完成：%s；仍按原触发原因停车：%s",
        error.what(), trigger.what());
    }
    throw;
  }

  const auto sample_cost = [&field](const Eigen::Vector2d & p, CostLinearization & cost) {
      return field.sample(p, cost);
    };

  const auto sample_envelope = [&](const State & measured, const Control & command, auto & planes)
  {
      return field.sampleEnvelope(
          measured,
          command,
          control_delay_,
          config_.control_period,
          config_.dt,
          planes,
          escape_contacts);
  };

  const auto sample_motion = [&](const State & a, const State & b, auto & planes)
  {
      return field.sampleMotion(
          a,
          b,
          config_.dt,
          planes,
          escape_contacts);
  };

  CommandModel command_model;
  command_model.phase = current.omega * output_delay_;
  command_model.deadzone = recovery_mode_ ? 0.0 : deadzone_; // Recovery 时禁用死区，避免低初速度被清零。
  command_model.fixed_angular_velocity = small_gyro_mode_;
  command_model.fixed_wz = fixed_wz_;
  recoveryStage("solver");
  if (!solver_->solve(current, reference, control, &prediction, sample_cost, &observed,
    sample_envelope, command_model, sample_motion))
  {
    throw ControllerInfeasible("局部控制求解失败：" + solver_->lastStatusMessage() +
      "（状态码=" + std::to_string(solver_->lastStatus()) + "）");
  }
  // QP 已输出最终全局速度；不再在求解后做相位旋转、限幅或死区截断。
  recoveryStage("prediction_safety");
  {
    RecoveryTimer prediction_timer(RecoveryMetric::PREDICTED_PATH);
    prediction.insert(prediction.begin(), observed);
    checkPrediction(prediction, "预测轨迹检查", field);
  }
  State next = current;
  next.x += control.vx * config_.dt;
  next.y += control.vy * config_.dt;
  next.yaw += control.omega * config_.dt;
  {
    RecoveryTimer final_timer(RecoveryMetric::FINAL);
    recoveryStage("final_safety");
    checkPrediction({current, next}, "最终输出指令检查", field);
    // 同时覆盖平移和转动，以及 [0, delay] 内所有执行延迟，不再使用外接圆。
    try {
      if (recovery_mode_) {
        if (std::abs(control.omega) > 1e-6) {
          throw std::runtime_error("Recovery MPC 不允许主动旋转");
        }
        field.checkEscapeEnvelope(observed, control, control_delay_,
          config_.control_period, recovery_contacts_.cells);
        const double braking = std::min({recovery_acceleration_ / std::sqrt(2.0),
          config_.ax_max, -config_.ax_min, config_.ay_max, -config_.ay_min});
        const double alpha = std::min(config_.alpha_max, -config_.alpha_min);
        // 撤销当前指令时仍存在输入延迟和一个控制周期，不能按瞬时停车检查。
        field.checkBrakingMotion(observed, control_delay_ + config_.control_period,
          braking, alpha, recovery_contacts_.cells);
        // 对所有可能的执行延迟，验证指令保持后再等待停车生效的制动轨迹。
        const double extent = map_server::calculateMinAndMaxDistances(footprint).second;
        const double travel = control_delay_ * (std::hypot(observed.vx, observed.vy) +
          std::hypot(control.vx, control.vy) + extent *
          (std::abs(observed.omega) + std::abs(control.omega)));
        const double samples = std::ceil(travel / std::min(0.025, snapshot.geometry().resolution * 0.5));
        if (!std::isfinite(samples) || samples > 10000) {
          throw std::runtime_error("Recovery 输出制动分支超出预算");
        }
        const int steps = std::max(1, static_cast<int>(samples));
        for (int i = 0; i <= steps; ++i) {
          const double delay = control_delay_ * i / steps;
          State held = observed;
          held.x += observed.vx * delay + control.vx * (control_delay_ + config_.control_period - delay);
          held.y += observed.vy * delay + control.vy * (control_delay_ + config_.control_period - delay);
          held.yaw += observed.omega * delay + control.omega * (control_delay_ + config_.control_period - delay);
          held.vx = control.vx;
          held.vy = control.vy;
          held.omega = control.omega;
          field.checkBrakingMotion(held, control_delay_ + config_.control_period,
            braking, alpha, recovery_contacts_.cells);
        }
      } else {
        field.checkEnvelope(observed, control, control_delay_, config_.control_period,
          "指令保持期间的轮廓安全检查");
      }
    } catch (const std::runtime_error & error) {throw ControllerInfeasible(error.what());}
  }
  // 本周期只使用冻结快照完成碰撞检查；正常换图留给下一周期，不撤销成功求解。
  // 发布前仍要求最新地图健康、消息有效且生产者未重启。
  if (!costmap_->isCurrent() ||
    costmap_->view()->status().producer_id != snapshot.status().producer_id)
  {
    throw std::runtime_error("求解期间地图或传感器状态已失效，拒绝输出速度");
  }
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node->now();
  command.header.frame_id = costmap_->getBaseFrameID();
  command.twist.linear.x = c * control.vx + s * control.vy;
  command.twist.linear.y = -s * control.vx + c * control.vy;
  command.twist.angular.z = control.omega;
  return command;  // Only PathExecutor publishes the command.
}
}  // namespace minco_controller
