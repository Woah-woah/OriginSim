#include "nav_executor/path_executor.hpp"
#include "nav_executor/odometry_observation.hpp"

#include <cmath>
#include <atomic>
#include <mutex>
#include <optional>
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
    Observation observation)
      : node_(node), costmap_(std::move(costmap)), global_ready_(std::move(global_ready)),
        failure_(std::move(failure)), success_(std::move(success)),
        confirmation_(std::move(confirmation)), progress_(std::move(progress)),
        observation_(std::move(observation)), logger_(node->get_logger()), clock_(node->get_clock())
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
    timer_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), [this]() {
      tick();
    });
    state_ = ExecutionState::IDLE;
  }

  ~Impl()
  {
    timer_.reset();
    publishZero();
  }

  void publishZero()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void emergencyStop()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    emergency_stop_ = true;
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
    cmd_pub_->publish(emergency_stop_ ? geometry_msgs::msg::Twist{} : command);
  }

  void stop(const std::string & reason)
  {
    if (state_ == ExecutionState::RUNNING) {
      RCLCPP_INFO(logger_, "路径执行已停止，原因：%s", reason.c_str());
    }
    path_.reset();
    completion_wait_since_.reset();
    publishZero(); // 先停车再重置优化器
    controller_.reset();
    state_ = ExecutionState::STOPPED;
  }

  void fail(const std::string & reason)
  {
    const bool had_path = path_.has_value();
    path_.reset();
    publishZero();
    if (had_path) {
      controller_.reset();
      state_ = ExecutionState::FAILED;
      RCLCPP_WARN(logger_, "路径执行已停止，原因：%s", reason.c_str());
      failure_(identity_, reason);
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
      failure_(identity_, exception.what());
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
      stop("急停已触发");
      return;
    }
    if (!confirmation_(identity_)) {
      stop("当前执行路径已被任务管理器撤销");
      return;
    }
    try {
      if (!global_ready_() || !costmap_->isCurrent()) {
        throw std::runtime_error("全局或局部地图不可用，或地图数据已过期");
      }
      geometry_msgs::msg::PoseStamped pose;
      if (!costmap_->getRobotPose(pose)) {
        throw std::runtime_error("无法取得机器人当前位置的坐标变换");
      }
      auto observation = pose.header.stamp;
      const auto speed = readSpeed("求解前", &observation);
      if (!costmap_->getRobotPose(pose, observation)) {
        throw std::runtime_error("无法取得机器人当前位置的坐标变换");
      }
      checkStamp(pose.header.stamp, tf_timeout_, "机器人坐标变换", true);
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
      if (!global_ready_() || !costmap_->isCurrent()) {
        throw std::runtime_error("控制计算期间地图已失效，拒绝发布速度");
      }
      // 在已确认的全局路径坐标系中上报有效反馈，监控回调可同步撤销路径
      observation_(identity_, route_pose, command.twist);
      if (!path_ || !confirmation_(identity_)) {
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
      publishCommand(command.twist);
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
  std::atomic<bool> emergency_stop_{false};
  std::mutex command_mutex_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double frequency_;
  double odom_timeout_;
  double tf_timeout_;
  double control_timeout_;
  GoalTolerances tolerances_;
  std::optional<TaskManager::Clock::time_point> completion_wait_since_;
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
  Observation observation)
    : impl_(std::make_unique<Impl>(node,
        std::move(local_costmap),
        std::move(global_map_ready),
        std::move(failure),
        std::move(success),
        std::move(confirmation),
        std::move(progress),
        std::move(observation)))
{
}

PathExecutor::~PathExecutor() = default;

bool PathExecutor::acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity)
{
  return impl_->acceptPath(path, identity);
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
