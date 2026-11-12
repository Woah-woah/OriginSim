#include "nav_executor/path_executor.hpp"
#include "nav_executor/odometry_observation.hpp"
#include "nav_executor/recovery.hpp"
#include "rclcpp/create_timer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cmath>
#include <atomic>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "minco_controller/minco_mpc_controller.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_executor
{
bool finiteTwist(const geometry_msgs::msg::Twist & speed)
{
  return std::isfinite(speed.linear.x) && std::isfinite(speed.linear.y) &&
         std::isfinite(speed.linear.z) && std::isfinite(speed.angular.x) &&
         std::isfinite(speed.angular.y) && std::isfinite(speed.angular.z);
}

geometry_msgs::msg::Twist transformRobotVelocity(const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::TransformStamped & base_from_child)
{
  if (!finiteTwist(speed)) {
    throw std::invalid_argument("里程计速度含无效数值");
  }
  tf2::Transform transform;
  tf2::fromMsg(base_from_child.transform, transform);
  const tf2::Vector3 angular =
    transform.getBasis() * tf2::Vector3(speed.angular.x, speed.angular.y, speed.angular.z);
  const tf2::Vector3 linear =
    transform.getBasis() * tf2::Vector3(speed.linear.x, speed.linear.y, speed.linear.z) +
    transform.getOrigin().cross(angular);
  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x();
  result.linear.y = linear.y();
  result.linear.z = linear.z();
  result.angular.x = angular.x();
  result.angular.y = angular.y();
  result.angular.z = angular.z();
  if (!finiteTwist(result)) {
    throw std::runtime_error("里程计速度转换到车体坐标系后无效");
  }
  return result;
}

namespace
{
struct GoalTolerances
{
  double xy{0.3};
  double yaw{3.14};
  double stopped_speed{0.05};
};

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
         std::abs(norm - 1.0) < 0.01;
}

} // 匿名命名空间

class PathExecutor::Impl
{
public:
  Impl(const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<map_server::MapClient> costmap,
    std::function<bool()> global_ready,
    Failure failure,
    Success success,
    Confirmation confirmation,
    Progress progress,
    Observation observation,
    RecoveryDone recovery_done)
      : node_(node), costmap_(std::move(costmap)), global_ready_(std::move(global_ready)),
        failure_(std::move(failure)), success_(std::move(success)),
        confirmation_(std::move(confirmation)), progress_(std::move(progress)),
        observation_(std::move(observation)), recovery_done_(std::move(recovery_done)), logger_(node->get_logger()), clock_(node->get_clock())
  {
    frequency_ = node->declare_parameter("controller_frequency", 20.0);
    odom_timeout_ = node->declare_parameter("odom_timeout", 0.5);
    tf_timeout_ = node->declare_parameter("tf_timeout", 0.5);
    control_timeout_ = node->declare_parameter("control_timeout", 0.2);
    tolerances_.xy = node->declare_parameter("xy_goal_tolerance", 0.3);
    tolerances_.yaw = node->declare_parameter("yaw_goal_tolerance", 3.14);
    tolerances_.stopped_speed = node->get_parameter("stopped_velocity_tolerance").as_double();
    const auto odom_topic = node->declare_parameter("odom_topic", std::string("/Odometry"));
    const auto cmd_topic = node->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    for (const auto value : {frequency_,
           odom_timeout_,
           tf_timeout_,
           control_timeout_,
           tolerances_.xy,
           tolerances_.yaw,
           tolerances_.stopped_speed})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
          "PathExecutor frequencies, timeouts and tolerances must be positive");
      }
    }
    if (frequency_ > 1000.0 || odom_topic.empty() || cmd_topic.empty()) {
      throw std::invalid_argument("路径执行器的控制频率或话题配置无效");
    }
    cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(1));
    publishZero();

    controller_.initialize(node, "MPC", costmap_);
    recovery_config_ = RecoveryConfig::load(node);
    recovery_alpha_ = std::min(node->get_parameter("MPC.alpha_max").as_double(),
      -node->get_parameter("MPC.alpha_min").as_double());
    if (!std::isfinite(recovery_alpha_) || recovery_alpha_ <= 0) {
      throw std::invalid_argument("Recovery 需要有效角减速度限制");
    }
    const auto debug_qos = rclcpp::QoS(1).reliable().transient_local();
    recovery_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nav_executor/recovery_candidates", debug_qos);
    recovery_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "nav_executor/recovery_path", debug_qos);
    // 输入接收独立于控制定时器，任务与控制器状态仍由默认回调组修改
    odom_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_group_;
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        try {
          const auto stamp = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
          if (msg->child_frame_id.empty() || stamp.nanoseconds() <= 0 ||
              !finiteTwist(msg->twist.twist) ||
              (odom_ && stamp < rclcpp::Time(odom_->header.stamp, clock_->get_clock_type())))
          {
            throw std::invalid_argument("里程计数据无效或时间顺序错误");
          }
          if (odom_ && (odom_->child_frame_id != msg->child_frame_id ||
              odom_->header.frame_id != msg->header.frame_id)) {odom_history_.clear();}
          if (!odom_history_.empty() && odom_history_.back().header.stamp == msg->header.stamp) {
            odom_history_.back() = *msg;
          } else {odom_history_.push_back(*msg);}
          while (odom_history_.size() > 2 && (odom_history_.size() > 256 ||
              (stamp - rclcpp::Time(odom_history_.front().header.stamp,
              clock_->get_clock_type())).seconds() > odom_timeout_)) {odom_history_.pop_front();}
          odom_ = *msg;
          odom_received_ = TaskManager::Clock::now();
        } catch (const std::exception & exception) {
          odom_.reset();
          odom_history_.clear();
          // 错误处理与控制器重置交由串行控制回调执行
          odom_error_ = exception.what();
        }
      },
      odom_options);
    // 控制周期与预测模型使用同一 ROS 时钟，仿真变慢或暂停时不能按墙钟多发加速步。
    timer_ = rclcpp::create_timer(node->get_node_base_interface(),
      node->get_node_timers_interface(), clock_, rclcpp::Duration::from_seconds(1.0 / frequency_),
      [this]() {tick();});
    watchdog_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), [this]() {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (drive_active_ && std::chrono::duration<double>(TaskManager::Clock::now() - command_at_).count() > control_timeout_) {
        const double ros_age = (clock_->now().nanoseconds() - command_ros_ns_) * 1e-9;
        const bool clock_paused = clock_->ros_time_is_active() &&
          (ros_age < control_timeout_ || ros_age < 0);
        drive_active_ = false;
        zero_command_published_ = true;
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
        if (clock_paused) {
          RCLCPP_WARN(logger_, "仿真时钟暂停、变慢或重置，已发布零速，等待时钟和输入恢复");
        } else {
          emergency_stop_ = true;
          RCLCPP_ERROR(logger_, "控制发布看门狗超时，已锁定急停");
        }
      }
    }, watchdog_group_);
    state_ = ExecutionState::IDLE;
  }

  ~Impl()
  {
    watchdog_.reset();
    timer_.reset();
    publishZero();
  }

  void publishZero()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    drive_active_ = false;
    zero_command_published_ = true;
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void emergencyStop()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    emergency_stop_ = true;
    drive_active_ = false;
    zero_command_published_ = true;
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void clearEmergencyStop()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    emergency_stop_ = false;
  }

  void publishCommand(const geometry_msgs::msg::Twist & command)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_at_ = TaskManager::Clock::now();
    command_ros_ns_ = clock_->now().nanoseconds();
    drive_active_ = !emergency_stop_ && (std::hypot(command.linear.x, command.linear.y) > 0 ||
      std::abs(command.angular.z) > 0);
    zero_command_published_ = !drive_active_;
    cmd_pub_->publish(emergency_stop_ ? geometry_msgs::msg::Twist{} : command);
  }

  void stop(const std::string & reason)
  {
    if (state_ == ExecutionState::RUNNING) {
      RCLCPP_INFO(logger_, "路径执行已停止，原因：%s", reason.c_str());
    }
    recovery_phase_ = RecoveryPhase::NONE;
    recovery_contacts_ = {};
    map_update_since_.reset();
    path_.reset();
    completion_wait_since_.reset();
    publishZero(); // 先停车再重置优化器
    controller_.reset();
    state_ = ExecutionState::STOPPED;
  }

  void fail(const std::string & reason, ExecutionFailure kind = ExecutionFailure::SAFETY_OR_INPUT)
  {
    if (minco_controller::active_recovery_profile) {minco_controller::active_recovery_profile->failed = true;}
    if (recovery_phase_ != RecoveryPhase::NONE) {completeRecovery(false, reason); return;}
    const bool had_path = path_.has_value();
    path_.reset();
    publishZero();
    if (had_path) {
      controller_.reset();
      state_ = ExecutionState::FAILED;
      RCLCPP_WARN(logger_, "路径执行已停止，原因：%s", reason.c_str());
      failure_(identity_, reason, kind);
    }
  }

  bool acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity)
  {
    if (emergency_stop_) {
      publishZero();
      return false;
    }
    if (!confirmation_(identity)) {
      RCLCPP_WARN(logger_,
        "忽略尚未确认的路径：目标编号=%lu，路径版本=%lu",
        static_cast<unsigned long>(identity.goal_id),
        static_cast<unsigned long>(identity.plan_generation));
      return false;
    }
    stop("切换到新执行路径");
    identity_ = identity;
    try {
      if (path.poses.empty() || path.header.frame_id.empty()) {
        throw std::invalid_argument("无法执行空路径或缺少坐标系的路径");
      }
      for (const auto & pose : path.poses) {
        if (!finitePose(pose.pose) ||
            (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
        {
          throw std::invalid_argument("执行路径包含无效位姿或坐标系不一致");
        }
      }
      controller_.setPlan(path);
      path_ = path;
      RCLCPP_INFO(logger_,
        "开始执行完整路径：目标编号=%lu，路径版本=%lu，路径点数=%zu",
        static_cast<unsigned long>(identity_.goal_id),
        static_cast<unsigned long>(identity_.plan_generation),
        path.poses.size());
      state_ = ExecutionState::RUNNING;
      return true;
    } catch (const std::exception & exception) {
      path_.reset();
      publishZero();
      state_ = ExecutionState::FAILED;
      failure_(identity_, exception.what(), ExecutionFailure::SAFETY_OR_INPUT);
      return false;
    }
  }

  void checkStamp(const builtin_interfaces::msg::Time & stamp,
    double timeout,
    const std::string & description,
    bool allow_zero = false) const
  {
    const rclcpp::Time time(stamp, clock_->get_clock_type());
    if (allow_zero && time.nanoseconds() == 0) {
      return;
    } // 静态坐标变换不受时间限制
    const auto now = clock_->now();
    const double age = (now - time).seconds();
    if (time.nanoseconds() <= 0 || age > timeout || age < -0.1) {
      throw std::runtime_error(
        description + " timestamp is stale or invalid: age=" + std::to_string(age) +
        "s timeout=" + std::to_string(timeout) + "s now=" + std::to_string(now.seconds()) +
        " stamp=" + std::to_string(time.seconds()));
    }
  }

  geometry_msgs::msg::Twist readSpeed(const std::string & phase = "求解前",
    builtin_interfaces::msg::Time * observation = nullptr)
  {
    std::optional<nav_msgs::msg::Odometry> odom;
    TaskManager::Clock::time_point received;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_error_.empty()) {
        const auto error = std::exchange(odom_error_, std::string{});
        throw std::runtime_error(error);
      }
      odom = odom_;
      received = odom_received_;
    }
    if (!odom ||
        std::chrono::duration<double>(TaskManager::Clock::now() - received).count() > odom_timeout_)
    {
      throw std::runtime_error("未收到里程计数据，或接收已超时");
    }
    try {
      checkStamp(odom->header.stamp, odom_timeout_, "里程计");
    } catch (const std::runtime_error & exception) {
      throw std::runtime_error(
        std::string(exception.what()) + "，检查阶段=" + phase + "，距上次接收已过=" +
        std::to_string(
          std::chrono::duration<double>(TaskManager::Clock::now() - received).count()) +
        "s");
    }
    // 求解后的调用只验证新鲜度，不使用返回值，无需再次做坐标变换。
    if (!observation) {return odom->twist.twist;}
    int64_t shared_time = observationNanoseconds(odom->header.stamp);
    const auto limit = [&](const builtin_interfaces::msg::Time & stamp) {
        const int64_t ns = observationNanoseconds(stamp);
        if (ns > 0) {shared_time = std::min(shared_time, ns);}
      };
    limit(*observation);
    if (odom->child_frame_id != costmap_->getBaseFrameID()) {
      const auto common = costmap_->getTfBuffer()->lookupTransform(
        costmap_->getBaseFrameID(), odom->child_frame_id, tf2::TimePointZero);
      limit(common.header.stamp);
    }
    const rclcpp::Time time(shared_time, clock_->get_clock_type());
    *observation = time;
    if (auto * profile = minco_controller::active_recovery_profile) {
      profile->pose_stamp = time.seconds();
      profile->aligned_age = (clock_->now() - time).seconds();
    }
    checkStamp(*observation, odom_timeout_, "共同观测时间的里程计");
    std::optional<geometry_msgs::msg::Twist> sampled;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      sampled = interpolateOdometryTwist(odom_history_, shared_time, odom_timeout_);
    }
    if (!sampled || !finiteTwist(*sampled)) {
      throw std::runtime_error("TF 与里程计尚无共同有效时间，拒绝外推实测速度");
    }
    if (odom->child_frame_id == costmap_->getBaseFrameID()) {return *sampled;}
    const auto transform = costmap_->getTfBuffer()->lookupTransform(
      costmap_->getBaseFrameID(), odom->child_frame_id, time);
    return transformRobotVelocity(*sampled, transform);
  }

  void reportCompletionWait(
    double distance, double yaw_error, const geometry_msgs::msg::Twist & speed)
  {
    const auto now = TaskManager::Clock::now();
    if (!completion_wait_since_) {
      completion_wait_since_ = now;
    }
    if (now - *completion_wait_since_ < std::chrono::seconds(2)) {
      return;
    }
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "尚未满足到点条件：距离误差=%.3f米（允许%.3f米），朝向误差=%.3f弧度（允许%.3f弧度）；"
      "实测速度=%.3f米/秒，实测角速度=%.3f弧度/秒，停止阈值=%.3f",
      distance,
      tolerances_.xy,
      std::abs(yaw_error),
      tolerances_.yaw,
      std::hypot(speed.linear.x, speed.linear.y),
      std::abs(speed.angular.z),
      tolerances_.stopped_speed);
  }

  bool startRecovery(const nav_msgs::msg::Path & route, PlanIdentity identity, bool new_episode)
  {
    stop("进入 Recovery 停车确认");
    if (!recovery_config_.enabled || emergency_stop_ || !confirmation_(identity) ||
      route.poses.empty()) {return false;}
    if (new_episode || recovery_goal_ != identity.goal_id) {
      recovery_goal_ = identity.goal_id;
      recovery_distance_ = 0;
      recovery_endpoints_.clear();
    }
    if (recovery_distance_ >= recovery_config_.total_distance) {return false;}
    identity_ = identity;
    path_ = route;
    recovery_phase_ = RecoveryPhase::STOPPING;
    recovery_started_ = TaskManager::Clock::now();
    recovery_phase_started_ = recovery_started_;
    recovery_still_since_.reset();
    recovery_previous_.reset();
    recovery_observation_ns_ = 0;
    recovery_observed_speed_ = 0;
    recovery_attempt_distance_ = 0;
    recovery_overspeed_since_.reset();
    recovery_contacts_ = {};
    recovery_plan_ = {};
    recovery_command_speed_ = 0.0;
    state_ = ExecutionState::RECOVERING;
    RCLCPP_INFO(logger_, "Recovery 地图模式：%s",
      costmap_->get_parameter("require_cloud").as_bool() ?
      "动态地图，要求实时点云有效" : "静态地图 EDT，未启用动态障碍观测");
    return true;
  }

  void completeRecovery(bool success, const std::string & reason)
  {
    publishZero();
    recovery_phase_ = RecoveryPhase::NONE;
    recovery_contacts_ = {};
    path_.reset();
    controller_.reset();
    state_ = success ? ExecutionState::STOPPED : ExecutionState::FAILED;
    RCLCPP_WARN(logger_, "Recovery %s：%s；累计实际距离=%.3f米",
      success ? "完成" : "拒绝/失败", reason.c_str(), recovery_distance_);
    recovery_done_(identity_, success, reason);
  }

  void visualizeRecovery(const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const auto & candidate : recovery_plan_.candidates) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "recovery_candidates";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.012;
      marker.color.b = 1;
      marker.color.g = 0.5;
      marker.color.a = 0.35;
      for (const auto & state : candidate.states) {
        geometry_msgs::msg::Point point;
        point.x = state.x;
        point.y = state.y;
        marker.points.push_back(point);
      }
      markers.markers.push_back(std::move(marker));
    }
    recovery_markers_->publish(markers);
    recovery_path_pub_->publish(recoveryPath(recovery_plan_.selected, header));
  }


  void tickRecovery(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    const builtin_interfaces::msg::Time & observation_stamp)
  {
    minco_controller::RecoveryTimer recovery_timer(minco_controller::RecoveryMetric::RECOVERY);
    minco_controller::recoveryStage("executor_checks");
    if (auto * profile = minco_controller::active_recovery_profile) {
      profile->entry_age = (clock_->now() - rclcpp::Time(pose.header.stamp,
        clock_->get_clock_type())).seconds();
    }
    using State = minco_controller::State;
    const auto began = TaskManager::Clock::now();
    const auto & c = recovery_config_;

    if (std::chrono::duration<double>(began - recovery_started_).count() > c.timeout) {
      throw std::runtime_error("Recovery 总超时");
    }

    const auto captured = costmap_->view();
    if (!costmap_->isCurrent(*captured)) {
      if (costmap_->isUpdating()) {throw minco_controller::MapChanged("Recovery 等待地图版本配对");}
      throw std::runtime_error("Recovery 地图失效");
    }
    // isCurrent 已按 MapClient 的模式校验健康：动态模式必须有有效点云，
    // 静态模式允许没有点云，后续仍使用同一真实轮廓连续扫掠及制动安全检查。
    const auto footprint = costmap_->footprintInBaseFrame(pose.header.stamp);
    const minco_controller::TrajectorySafety safety(*captured, footprint, c.collision_margin);
    const double yaw = tf2::getYaw(pose.pose.orientation);
    State observed;
    observed.x = pose.pose.position.x;
    observed.y = pose.pose.position.y;
    observed.yaw = yaw;
    observed.vx = std::cos(yaw) * speed.linear.x - std::sin(yaw) * speed.linear.y;
    observed.vy = std::sin(yaw) * speed.linear.x + std::cos(yaw) * speed.linear.y;
    observed.omega = speed.angular.z;
    const Eigen::Vector2d position(observed.x, observed.y);
    const double measured = std::hypot(observed.vx, observed.vy);
    if (recovery_previous_ && (position - *recovery_previous_).norm() > c.speed * control_timeout_ + c.tracking_tolerance)
    {
      throw std::runtime_error("Recovery 定位跳变");
    }
    recovery_previous_ = position;
    const int64_t observation_ns = rclcpp::Time(observation_stamp, clock_->get_clock_type()).nanoseconds();
    if (recovery_observation_ns_ > 0) {
      const double dt = (observation_ns - recovery_observation_ns_) * 1e-9;
      if (dt < 0 || dt > odom_timeout_) {throw std::runtime_error("Recovery 里程计观测时间不连续");}
      // 物理行驶距离由共同观测时刻的里程计速度积分；不累加定位修正的毫米级抖动。
      // 初次停车是上一段导航的制动，不占用尚未开始的脱困平移距离预算。
      if (recovery_phase_ != RecoveryPhase::STOPPING) {
        const double distance = 0.5 * (recovery_observed_speed_ + measured) * dt;
        recovery_distance_ += distance;
        recovery_attempt_distance_ += distance;
      }
    }
    recovery_observation_ns_ = observation_ns;
    recovery_observed_speed_ = measured;
    if (recovery_distance_ > c.total_distance || recovery_attempt_distance_ > c.max_distance) {
      throw std::runtime_error("Recovery 实际累计行驶距离超限");
    }
    const bool stopped = measured <= tolerances_.stopped_speed &&
      std::abs(observed.omega) <= tolerances_.stopped_speed;
    const auto still = [&]() {
        if (!stopped) {recovery_still_since_.reset(); return false;}
        if (!recovery_still_since_) {recovery_still_since_ = began;}
        return std::chrono::duration<double>(began - *recovery_still_since_).count() >= c.settle_time;
      };
    if (recovery_phase_ == RecoveryPhase::STOPPING || recovery_phase_ == RecoveryPhase::FINISHING) {
      publishZero();
      if (std::chrono::duration<double>(began - recovery_phase_started_).count() > c.stop_timeout) {
        throw std::runtime_error("Recovery 停车确认超时");
      }
      // 停车阶段只发布零速，不能拿低速脱困的制动距离拒绝上一段导航的余速。
      // 必须取得连续停稳证据后才检查起始轮廓、选方向并允许新的平移动作。
      if (!still()) {
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
          "Recovery 等待停稳：平移速度=%.3f米/秒，角速度=%.3f弧度/秒",
          measured, std::abs(observed.omega));
        return;
      }

      // FINISHING 阶段仍然需要严格检查
      if (recovery_phase_ == RecoveryPhase::FINISHING)
      {
          safety.checkPose(observed, "Recovery 停稳后轮廓");

          recovery_endpoints_.push_back(position);
          completeRecovery(true, "脱困后已停稳，请求保留目标的全局重规划");
          return;
      }

      // 停稳时冻结初始集合；失败或越界时禁止启动运动。
      recovery_contacts_.initialize(*captured, safety, observed);
      if (recovery_contacts_.count > 0) {
        RCLCPP_WARN(logger_, "Recovery 从初始重叠启动，冻结接触栅格数量=%d", recovery_contacts_.count);
      }

      // 在控制坐标系中冻结真实车体轮廓；路径反向独立于车头方向。
      Eigen::Vector2d reverse = Eigen::Vector2d::Zero();
      double nearest = INFINITY;
      auto route = *path_;
      for (auto & waypoint : route.poses) {
        waypoint.header.frame_id = route.header.frame_id;
        waypoint.header.stamp = pose.header.stamp;
        waypoint = costmap_->getTfBuffer()->transform(waypoint, pose.header.frame_id);
      }
      for (size_t i = 1; i < route.poses.size(); ++i) {
        const auto & a = route.poses[i - 1].pose.position;
        const auto & b = route.poses[i].pose.position;
        const Eigen::Vector2d direction(a.x - b.x, a.y - b.y);
        const double distance = (position - Eigen::Vector2d(a.x, a.y)).squaredNorm();
        if (distance < nearest && direction.norm() > 1e-6) {nearest = distance; reverse = direction.normalized();}
      }
      auto search_config = c;
      search_config.max_distance = std::min(c.max_distance - recovery_attempt_distance_, c.total_distance - recovery_distance_);
      if (search_config.max_distance < 0.1) {throw std::runtime_error("Recovery 剩余距离预算不足");}

      recovery_plan_ = RecoveryPlanner(search_config).plan(*captured, footprint, observed, reverse, recovery_endpoints_);
      // 将 Recovery 搜索结果转换为 MPC 可以执行的路径
      auto mpc_path = recoveryPath(recovery_plan_.selected, pose.header);
      visualizeRecovery(pose.header);
      controller_.setPlan(mpc_path, c.speed, c.acceleration);
      controller_.setRecoveryContacts(recovery_contacts_);

      recovery_previous_ = position;
      recovery_progress_anchor_ = position;
      recovery_progress_at_ = began;
      recovery_phase_started_ = began;
      recovery_phase_ = RecoveryPhase::MOVING;
      recovery_command_speed_ = 0.0;
      RCLCPP_INFO(logger_, "Recovery 选择轨迹：地图=%llu，候选=%zu，评分=%.3f，最小轮廓净空=%.3f米，"
        "长度=%.3f米，MPC 速度上限=%.3f米/秒（单方向平移）", static_cast<unsigned long long>(captured->version()),
        recovery_plan_.candidates.size(), recovery_plan_.selected.score,
        recovery_plan_.selected.clearance, recovery_plan_.selected.length, c.speed);
      return;
    }

    if (captured->status().producer_id != recovery_plan_.producer) {
      throw std::runtime_error("Recovery 地图生产端重启");
    }
    recovery_contacts_.update(*captured, safety, observed);
    const auto check_motion = [&](const State & a, const State & b, const std::string & stage) {
        if (!safety.checkEscapeMotion(a, b, &recovery_contacts_.cells)) {
          throw std::runtime_error(stage + "：接触恶化或产生新接触");
        }
      };
    // 空集合自动走严格检测；初始重叠也必须验证惯性停车全过程。
    safety.checkBrakingMotion(observed, c.delay + 1.0 / frequency_,
      c.braking / std::sqrt(2.0), recovery_alpha_, recovery_contacts_.cells);
    // 实测超出低速跟踪阈值时立即撤销指令；允许停稳后复检继续，不放宽速度阈值。
    // 保留原方向、总超时和距离预算，不能借停车重置脱困上限。
    if (measured > c.speed + c.velocity_tolerance || recovery_overspeed_since_) {
      publishZero();
      const double previous_command = recovery_command_speed_;
      recovery_command_speed_ = 0.0;
      if (!recovery_overspeed_since_) {
        recovery_overspeed_since_ = began;
        recovery_still_since_.reset();
        RCLCPP_WARN(logger_, "Recovery 实测速度超出跟踪阈值，先停车：实测=%.3f米/秒，"
          "前一指令=%.3f米/秒，配置=%.3f米/秒，阈值=%.3f米/秒，角速度=%.3f弧度/秒",
          measured, previous_command, c.speed, c.speed + c.velocity_tolerance, observed.omega);
      }
      if (std::chrono::duration<double>(began - *recovery_overspeed_since_).count() > c.stop_timeout) {
        throw std::runtime_error("Recovery 实测速率异常后无法确认停稳");
      }
      if (still()) {
        recovery_overspeed_since_.reset();
        recovery_still_since_.reset();
        recovery_progress_anchor_ = position;
        recovery_progress_at_ = began;
        RCLCPP_INFO(logger_, "Recovery 速度异常后已停稳，下周期重新检查剩余轨迹并继续原方向");
      }
      return;
    }
    const auto & states = recovery_plan_.selected.states;
    if (states.size() != 2) {throw std::runtime_error("Recovery 需要一段直线平移轨迹");}
    auto target = states.back();
    target.yaw = yaw;
    // 固定已选方向，在每份新地图上复检整段剩余轨迹；换图不重新停车选方向。
    check_motion(observed, target, "Recovery 剩余轨迹实时复检");
    recovery_plan_.version = captured->version();
    recovery_plan_.dynamic_version = captured->status().dynamic_version;
    const Eigen::Vector2d error(target.x - observed.x, target.y - observed.y);
    const auto & segment_start = states.front();
    const Eigen::Vector2d segment(target.x - segment_start.x, target.y - segment_start.y);
    if (segment.squaredNorm() < 1e-12) {throw std::runtime_error("Recovery 平移轨迹长度无效");}
    const double t = std::clamp((position - Eigen::Vector2d(segment_start.x, segment_start.y)).dot(segment) /
      segment.squaredNorm(), 0.0, 1.0);
    if ((position - Eigen::Vector2d(segment_start.x, segment_start.y) - t * segment).norm() > c.tracking_tolerance) {
      throw std::runtime_error("Recovery 轨迹跟踪偏差超限");
    }
    if ((position - recovery_progress_anchor_).norm() >= c.minimum_progress) {
      recovery_progress_anchor_ = position;
      recovery_progress_at_ = began;
    } else if (std::chrono::duration<double>(began - recovery_progress_at_).count() > c.progress_timeout) {
      throw std::runtime_error("Recovery 执行无进展");
    }

    // 进入位置容差后先停车，FINISHING 再确认停稳和严格净空。
    if (error.norm() <= c.waypoint_tolerance)
    {
        publishZero();

        recovery_command_speed_ = 0.0;
        recovery_still_since_.reset();

        recovery_phase_ = RecoveryPhase::FINISHING;
        recovery_phase_started_ = began;

        return;
    }

    recovery_still_since_.reset();

    // 交给 MPC 计算速度
    if (zero_command_published_) {controller_.notifyZeroCommandPublished();}
    auto output = controller_.computeVelocityCommands(pose, speed).twist;

    // 检查 MPC 输出
    double output_speed = std::hypot(output.linear.x, output.linear.y);

    if (!finiteTwist(output) ||
        output_speed > c.speed + 1e-3 ||
        std::abs(output.angular.z) > 1e-3)
    {
        throw std::runtime_error("Recovery MPC 输出速度无效或超限");
    }

    // MPC 已验证最终输出的延迟包络和制动过程，执行器直接发布车体坐标系速度。
    minco_controller::recoveryStage("publish_inputs");
    if (auto * profile = minco_controller::active_recovery_profile) {
      profile->publish_check = true;
      profile->publish_age = (clock_->now() - rclcpp::Time(pose.header.stamp,
        clock_->get_clock_type())).seconds();
    }
    readSpeed("Recovery 发布前");
    if (auto * profile = minco_controller::active_recovery_profile) {
      profile->publish_age = (clock_->now() - rclcpp::Time(pose.header.stamp,
        clock_->get_clock_type())).seconds();
    }
    checkStamp(pose.header.stamp, tf_timeout_, "Recovery TF");
    const auto latest = costmap_->view();
    if (costmap_->isUpdating()) {throw minco_controller::MapChanged("Recovery 等待地图版本配对");}
    if (costmap_->isCurrent(*latest) && latest->status().producer_id == captured->status().producer_id &&
      (latest->version() != captured->version() ||
       latest->dynamicVersion() != captured->dynamicVersion()))
    {throw minco_controller::MapChanged("Recovery 计算期间地图更新，本周期停车并等待复检");}
    if (!global_ready_() || !costmap_->isCurrent(*latest) ||
      latest->status().producer_id != captured->status().producer_id ||
      std::chrono::duration<double>(TaskManager::Clock::now() - began).count() > control_timeout_)
    {throw std::runtime_error("Recovery 输入失效、地图生产端重启或控制超时");}

    recovery_command_speed_ = output_speed;
    publishCommand(output);
    if (auto * profile = minco_controller::active_recovery_profile) {profile->published = true;}
    minco_controller::recoveryStage("published");
  }

  void tick()
  {
    if (!path_) {
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_error_.clear();
      }
      publishZero();
      return;
    }
    if (emergency_stop_) {
      fail("急停或控制发布看门狗已触发");
      return;
    }
    if (!confirmation_(identity_)) {
      stop("当前执行路径已被任务管理器撤销");
      return;
    }
    // 周期诊断包含输入查询，正常导航不启用；计时在异常处理之后仍可报告。
    minco_controller::RecoveryProfileSession profile_session(
      recovery_phase_ != RecoveryPhase::NONE,
      [this](const minco_controller::RecoveryProfile & profile, bool unwinding) {
        using Metric = minco_controller::RecoveryMetric;
        const auto now = TaskManager::Clock::now();
        const bool failed = profile.failed || unwinding;
        // 按真实时间节流，仿真暂停时也不会高频打印；失败周期保留完整诊断。
        if (!failed && now - recovery_profile_logged_ < std::chrono::seconds(1)) {return;}
        recovery_profile_logged_ = now;
        const double ros_end = clock_->now().seconds();
        std::ostringstream line;
        line.setf(std::ios::fixed);
        line.precision(3);
        line << "[RecoveryPerf] phase=" << profile.phase << " stage=" << profile.stage << " failed=" << failed
             << " published=" << profile.published
             << " tf_timeout_s=" << tf_timeout_
             << " control_timeout_ms=" << control_timeout_ * 1000
             << " exceeded=" << (profile.get(Metric::RECOVERY).ms > control_timeout_ * 1000)
             << " tick_exceeded=" << (profile.get(Metric::TICK).ms > control_timeout_ * 1000)
             << " tf_input_age_s=" << profile.input_age << " tf_aligned_age_s=" << profile.aligned_age
             << " tf_entry_age_s=" << profile.entry_age
             << " tf_publish_age_s=" << profile.publish_age
             << " publish_check=" << profile.publish_check
             << " tf_end_age_s=" << ros_end - profile.pose_stamp
             << " ros_delta_s=" << ros_end - profile.ros_begin;
        const std::pair<const char *, Metric> metrics[] = {
          {"tick", Metric::TICK}, {"input", Metric::INPUT}, {"recovery", Metric::RECOVERY},
          {"mpc", Metric::MPC}, {"solve", Metric::SOLVE}, {"qp", Metric::QP},
          {"checkPrediction", Metric::PREDICTION}, {"prediction_path", Metric::PREDICTED_PATH},
          {"final", Metric::FINAL},
          {"collisionCellCount", Metric::COLLISION}, {"checkEscapeMotion", Metric::ESCAPE},
          {"sampleMotion", Metric::SAMPLE_MOTION}, {"sampleEnvelope", Metric::SAMPLE_ENVELOPE},
          {"braking", Metric::BRAKING}, {"checkMotion", Metric::MOTION},
          {"square", Metric::SQUARE}, {"hull", Metric::HULL},
          {"projection", Metric::PROJECTION}, {"appendPlane", Metric::APPEND},
          {"visitObstacles", Metric::VISIT}};
        for (const auto & metric : metrics) {
          const auto & timing = profile.get(metric.second);
          line << " " << metric.first << "=" << timing.calls << "/" << timing.ms << "ms";
        }
        line << " sample_obstacles=" << profile.sample_obstacles
             << " sample_obstacles_max=" << profile.sample_obstacles_max
             << " sample_planes=" << profile.sample_planes
             << " sample_planes_max=" << profile.sample_planes_max
             << " plane_comparisons=" << profile.plane_comparisons
             << " plane_duplicates=" << profile.plane_duplicates
             << " collision_cache_hits=" << profile.collision_cache_hits;
        RCLCPP_WARN(logger_, "%s", line.str().c_str());
      });
    minco_controller::RecoveryTimer tick_timer(minco_controller::RecoveryMetric::TICK);
    if (auto * profile = minco_controller::active_recovery_profile) {
      profile->ros_begin = clock_->now().seconds();
      profile->phase = recovery_phase_ == RecoveryPhase::STOPPING ? "STOPPING" :
        (recovery_phase_ == RecoveryPhase::MOVING ? "MOVING" : "FINISHING");
    }
    try {
      if (!global_ready_() || !costmap_->isCurrent()) {
        if (costmap_->isUpdating()) {throw minco_controller::MapChanged("等待地图新版本完成配对");}
        throw std::runtime_error("全局或局部地图不可用，或地图数据已过期");
      }
      geometry_msgs::msg::PoseStamped pose;
      geometry_msgs::msg::Twist speed;
      builtin_interfaces::msg::Time observation;
      {
        minco_controller::RecoveryTimer input_timer(minco_controller::RecoveryMetric::INPUT);
        if (!costmap_->getRobotPose(pose)) {
          // MapClient 会拒绝过期 TF；仅在失败时读取原始时间戳用于诊断，不用于控制。
          if (auto * profile = minco_controller::active_recovery_profile) {
            try {
              const auto tf = costmap_->getTfBuffer()->lookupTransform(
                costmap_->getGlobalFrameID(), costmap_->getBaseFrameID(), tf2::TimePointZero);
              profile->pose_stamp = rclcpp::Time(tf.header.stamp, clock_->get_clock_type()).seconds();
              profile->input_age = clock_->now().seconds() - profile->pose_stamp;
            } catch (const tf2::TransformException & error) {
              RCLCPP_DEBUG(logger_, "Recovery 原始 TF 时间诊断不可用：%s", error.what());
            }
          }
          throw std::runtime_error("无法取得机器人当前位置的坐标变换");
        }
        if (auto * profile = minco_controller::active_recovery_profile) {
          profile->pose_stamp = rclcpp::Time(pose.header.stamp, clock_->get_clock_type()).seconds();
          profile->input_age = clock_->now().seconds() - profile->pose_stamp;
        }
        observation = pose.header.stamp;
        speed = readSpeed("求解前", &observation);
        if (!costmap_->getRobotPose(pose, observation)) {
          throw std::runtime_error("无法取得机器人当前位置的坐标变换");
        }
        if (auto * profile = minco_controller::active_recovery_profile) {
          profile->pose_stamp = rclcpp::Time(pose.header.stamp, clock_->get_clock_type()).seconds();
        }
        checkStamp(pose.header.stamp, tf_timeout_, "机器人坐标变换", true);
      }
      if (!finitePose(pose.pose)) {throw std::runtime_error("Recovery/导航位姿无效");}
      if (recovery_phase_ != RecoveryPhase::NONE) {
        tickRecovery(pose, speed, observation);
        return;
      }
      // 检查全局路径与局部里程计坐标系的变换
      const auto path_tf = costmap_->getTfBuffer()->lookupTransform(
        costmap_->getGlobalFrameID(), path_->header.frame_id, tf2::TimePointZero);
      checkStamp(path_tf.header.stamp, tf_timeout_, "路径坐标变换", true);
      auto goal = path_->poses.back();
      goal.header.frame_id = path_->header.frame_id;
      goal.header.stamp = builtin_interfaces::msg::Time{};
      goal = costmap_->getTfBuffer()->transform(goal, costmap_->getGlobalFrameID());
      if (!finitePose(pose.pose) || !finitePose(goal.pose)) {
        throw std::runtime_error("机器人或目标的坐标变换结果无效");
      }
      auto route_pose = pose;
      route_pose.header.stamp = builtin_interfaces::msg::Time{};
      route_pose = costmap_->getTfBuffer()->transform(route_pose, path_->header.frame_id);
      progress_(identity_, route_pose);
      if (!confirmation_(identity_) || emergency_stop_) {
        publishZero();
        return;
      }
      const double distance = std::hypot(
        pose.pose.position.x - goal.pose.position.x, pose.pose.position.y - goal.pose.position.y);
      const double yaw_error = std::remainder(
        tf2::getYaw(pose.pose.orientation) - tf2::getYaw(goal.pose.orientation), 2.0 * M_PI);
      if (distance <= tolerances_.xy && std::abs(yaw_error) <= tolerances_.yaw) {
        publishZero();
        if (std::hypot(speed.linear.x, speed.linear.y) <= tolerances_.stopped_speed &&
            std::abs(speed.angular.z) <= tolerances_.stopped_speed)
        {
          path_.reset();
          controller_.reset();
          state_ = ExecutionState::SUCCEEDED;
          completion_wait_since_.reset();
          success_(identity_);
        } else {
          reportCompletionWait(distance, yaw_error, speed);
        }
        return;
      }
      const auto grid = costmap_->view();
      if (!costmap_->isCurrent(*grid) || !grid->cell({pose.pose.position.x, pose.pose.position.y})) {
        throw std::runtime_error("机器人位置超出可用地图或地图已过期");
      }
      const auto began = TaskManager::Clock::now();
      if (zero_command_published_) {controller_.notifyZeroCommandPublished();}
      auto command = controller_.computeVelocityCommands(pose, speed);
      if (std::chrono::duration<double>(TaskManager::Clock::now() - began).count() >
            control_timeout_ ||
          !finiteTwist(command.twist))
      {
        throw std::runtime_error("控制器输出无效，或本次求解超过允许时间");
      }
      // 计算期间输入可能过期，发布前重新校验
      checkStamp(pose.header.stamp, tf_timeout_, "机器人坐标变换", true);
      checkStamp(path_tf.header.stamp, tf_timeout_, "路径坐标变换", true);
      readSpeed("求解后");
      if (costmap_->isUpdating()) {throw minco_controller::MapChanged("发布前等待地图版本配对");}
      if (!global_ready_() || !costmap_->isCurrent() ||
        costmap_->view()->status().producer_id != grid->status().producer_id)
      {throw std::runtime_error("控制计算期间地图已失效，拒绝发布速度");}
      // 在已确认的全局路径坐标系中上报有效反馈，监控回调可同步撤销路径
      observation_(identity_, route_pose, command.twist);
      if (!path_ || recovery_phase_ != RecoveryPhase::NONE || !confirmation_(identity_) ||
        emergency_stop_)
      {
        publishZero();
        return;
      }
      if (distance <= std::max(0.3, 2.0 * tolerances_.xy) &&
          std::hypot(command.twist.linear.x, command.twist.linear.y) < 0.1 &&
          std::abs(command.twist.angular.z) < 0.1)
      {
        reportCompletionWait(distance, yaw_error, speed);
      } else {
        completion_wait_since_.reset();
      }
      // 反馈回调和发布前输入检查也计入控制结果的总时限。
      if (std::chrono::duration<double>(TaskManager::Clock::now() - began).count() > control_timeout_)
      {
        throw std::runtime_error("控制指令发布前已超时，拒绝发布速度");
      }
      map_update_since_.reset();
      publishCommand(command.twist);
    } catch (const minco_controller::MapChanged & exception) {
      if (recovery_phase_ != RecoveryPhase::NONE) {
        publishZero();
        recovery_command_speed_ = 0.0;
        if (std::chrono::duration<double>(TaskManager::Clock::now() - recovery_started_).count() >
          recovery_config_.timeout)
        {
          completeRecovery(false, "Recovery 等待地图更新期间总超时");
          return;
        }
        // 撤销本周期指令，保留阶段和已选直线；下一周期用新图完整复检后恢复。
        // 否则持续地图更新会不断重置停车计时，始终无法进入或完成脱困。
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "%s", exception.what());
      } else {
        // 正常换图只撤销本周期输出，保留已确认路径和目标，下一周期重新求解。
        publishZero();
        const auto now = TaskManager::Clock::now();
        if (!map_update_since_) {map_update_since_ = now;}
        if (std::chrono::duration<double>(now - *map_update_since_).count() > odom_timeout_) {
          fail("地图持续变化，无法获得稳定控制快照", ExecutionFailure::MAP_CHANGED);
        } else {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "%s", exception.what());
        }
      }
    } catch (const ConnectorTraversalInterrupted & exception) {
      fail(exception.what(), ExecutionFailure::CONNECTOR_TRAVERSAL_INTERRUPTED);
    } catch (const minco_controller::PredictedCollision & exception) {
      fail(exception.what(), ExecutionFailure::MPC_PREDICTION_COLLISION);
    } catch (const minco_controller::ControllerInfeasible & exception) {
      fail(exception.what(), ExecutionFailure::CONTROLLER_INFEASIBLE);
    } catch (const std::exception & exception) {
      fail(exception.what());
    }
  }

  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<map_server::MapClient> costmap_;
  std::function<bool()> global_ready_;
  Failure failure_;
  Success success_;
  Confirmation confirmation_;
  Progress progress_;
  Observation observation_;
  RecoveryDone recovery_done_;
  enum class RecoveryPhase { NONE, STOPPING, MOVING, FINISHING };
  RecoveryPhase recovery_phase_{RecoveryPhase::NONE};
  TaskManager::Clock::time_point recovery_profile_logged_{};
  RecoveryConfig recovery_config_;
  RecoveryPlan recovery_plan_;
  minco_controller::RecoveryContacts recovery_contacts_;
  double recovery_alpha_{0};
  uint64_t recovery_goal_{0};
  double recovery_distance_{0}, recovery_attempt_distance_{0};
  std::optional<Eigen::Vector2d> recovery_previous_;
  int64_t recovery_observation_ns_{0};
  double recovery_observed_speed_{0};
  double recovery_command_speed_{0.0};
  Eigen::Vector2d recovery_progress_anchor_{0, 0};
  std::vector<Eigen::Vector2d> recovery_endpoints_;
  TaskManager::Clock::time_point recovery_started_, recovery_phase_started_, recovery_progress_at_;
  std::optional<TaskManager::Clock::time_point> recovery_still_since_;
  std::optional<TaskManager::Clock::time_point> recovery_overspeed_since_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr recovery_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recovery_path_pub_;
  bool drive_active_{false};
  TaskManager::Clock::time_point command_at_{};
  int64_t command_ros_ns_{0};
  rclcpp::CallbackGroup::SharedPtr watchdog_group_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::atomic<bool> emergency_stop_{false};
  std::atomic<bool> zero_command_published_{true};
  std::mutex command_mutex_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double frequency_;
  double odom_timeout_;
  double tf_timeout_;
  double control_timeout_;
  GoalTolerances tolerances_;
  std::optional<TaskManager::Clock::time_point> completion_wait_since_;
  std::optional<TaskManager::Clock::time_point> map_update_since_;
  minco_controller::MincoMpcController controller_;
  std::optional<nav_msgs::msg::Path> path_;
  std::mutex odom_mutex_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::string odom_error_;
  rclcpp::CallbackGroup::SharedPtr odom_group_;
  TaskManager::Clock::time_point odom_received_{};
  PlanIdentity identity_;
  ExecutionState state_{ExecutionState::IDLE};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

PathExecutor::PathExecutor(const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<map_server::MapClient> local_costmap,
  std::function<bool()> global_map_ready,
  Failure failure,
  Success success,
  Confirmation confirmation,
  Progress progress,
  Observation observation,
  RecoveryDone recovery_done)
    : impl_(std::make_unique<Impl>(node,
        std::move(local_costmap),
        std::move(global_map_ready),
        std::move(failure),
        std::move(success),
        std::move(confirmation),
        std::move(progress),
        std::move(observation),
        std::move(recovery_done)))
{
}

PathExecutor::~PathExecutor() = default;

bool PathExecutor::acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity)
{
  return impl_->acceptPath(path, identity);
}

bool PathExecutor::startRecovery(const nav_msgs::msg::Path & route, PlanIdentity identity, bool new_episode)
{
  return impl_->startRecovery(route, identity, new_episode);
}

void PathExecutor::emergencyStop()
{
  impl_->emergencyStop();
}

void PathExecutor::clearEmergencyStop()
{
  impl_->clearEmergencyStop();
}

void PathExecutor::stop(const std::string & reason)
{
  impl_->stop(reason);
}

ExecutionState PathExecutor::state() const
{
  return impl_->state_;
}
} // 命名空间 nav_executor
