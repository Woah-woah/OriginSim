#include "nav_executor/path_executor.hpp"

#include <atomic>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "mppi_controller/controller.hpp"
#include "std_msgs/msg/string.hpp"
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

geometry_msgs::msg::Twist transformRobotVelocity(
  const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::TransformStamped & base_from_child)
{
  if (!finiteTwist(speed)) {throw std::invalid_argument("Odometry velocity is not finite");}
  tf2::Transform transform;
  tf2::fromMsg(base_from_child.transform, transform);
  const tf2::Vector3 angular = transform.getBasis() *
    tf2::Vector3(speed.angular.x, speed.angular.y, speed.angular.z);
  const tf2::Vector3 linear = transform.getBasis() *
    tf2::Vector3(speed.linear.x, speed.linear.y, speed.linear.z) +
    transform.getOrigin().cross(angular);
  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x();
  result.linear.y = linear.y();
  result.linear.z = linear.z();
  result.angular.x = angular.x();
  result.angular.y = angular.y();
  result.angular.z = angular.z();
  if (!finiteTwist(result)) {throw std::runtime_error("Invalid transformed odometry velocity");}
  return result;
}

namespace
{
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
}

const char * stateName(ExecutionState state)
{
  switch (state) {
    case ExecutionState::IDLE: return "IDLE";
    case ExecutionState::RUNNING: return "RUNNING";
    case ExecutionState::STOPPED: return "STOPPED";
    case ExecutionState::SUCCEEDED: return "SUCCEEDED";
    case ExecutionState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

class GoalTolerances : public mppi::GoalChecker
{
public:
  double xy{0.3};
  double yaw{3.14};
  double stopped_speed{0.05};
  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose.position.x = pose.position.y = xy;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);
    pose.orientation = tf2::toMsg(rotation);
    velocity.linear.x = velocity.linear.y = velocity.angular.z = stopped_speed;
    return true;
  }
};
}  // namespace

class PathExecutor::Impl
{
public:
  Impl(const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    std::function<bool()> global_ready, Failure failure, Success success,
    Confirmation confirmation, Observation observation)
  : node_(node), costmap_(std::move(costmap)), global_ready_(std::move(global_ready)),
    failure_(std::move(failure)), success_(std::move(success)),
    confirmation_(std::move(confirmation)), observation_(std::move(observation)),
    logger_(node->get_logger()), clock_(node->get_clock())
  {
    frequency_ = node->declare_parameter("controller_frequency", 20.0);
    odom_timeout_ = node->declare_parameter("odom_timeout", 0.5);
    tf_timeout_ = node->declare_parameter("tf_timeout", 0.5);
    control_timeout_ = node->declare_parameter("control_timeout", 0.2);
    tolerances_.xy = node->declare_parameter("xy_goal_tolerance", 0.3);
    tolerances_.yaw = node->declare_parameter("yaw_goal_tolerance", 3.14);
    tolerances_.stopped_speed = node->declare_parameter("stopped_velocity_tolerance", 0.05);
    const auto odom_topic = node->declare_parameter("odom_topic", std::string("/Odometry"));
    const auto cmd_topic = node->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    for (const auto value : {frequency_, odom_timeout_, tf_timeout_, control_timeout_,
      tolerances_.xy, tolerances_.yaw, tolerances_.stopped_speed})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("PathExecutor frequencies, timeouts and tolerances must be positive");
      }
    }
    if (frequency_ > 1000.0 || odom_topic.empty() || cmd_topic.empty()) {
      throw std::invalid_argument("Invalid PathExecutor frequency or topic");
    }
    cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(1));
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = node->create_publisher<std_msgs::msg::String>("nav_executor/execution_state", state_qos);
    error_pub_ = node->create_publisher<std_msgs::msg::String>("nav_executor/execution_error", state_qos);
    publishZero();

    mppi::CostmapConfig config;
    config.global_frame = costmap_->getGlobalFrameID();
    config.robot_base_frame = costmap_->getBaseFrameID();
    config.footprint = costmap_->getRobotFootprint();
    config.robot_radius = costmap_->get_parameter("robot_radius").as_double();
    config.track_unknown = costmap_->get_parameter("track_unknown_space").as_bool();
    const auto plugins = costmap_->get_parameter("plugins").as_string_array();
    for (const auto & plugin : plugins) {
      if (costmap_->get_parameter(plugin + ".plugin").as_string() ==
        "nav2_costmap_2d::InflationLayer" &&
        costmap_->get_parameter(plugin + ".enabled").as_bool())
      {
        config.inflation_radius = costmap_->get_parameter(plugin + ".inflation_radius").as_double();
        config.inflation_cost_scaling_factor =
          costmap_->get_parameter(plugin + ".cost_scaling_factor").as_double();
        break;
      }
    }
    // Reject open-loop operation: MPPI must use measured robot speed.
    if (!node->has_parameter("FollowPath.open_loop")) {
      node->declare_parameter("FollowPath.open_loop", false);
    }
    if (node->get_parameter("FollowPath.open_loop").as_bool()) {
      throw std::invalid_argument("FollowPath.open_loop must be false for measured odometry feedback");
    }
    controller_.initialize(node, "FollowPath", costmap_->getTfBuffer(), costmap_->getCostmap(), config);
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
        if (msg->child_frame_id.empty() || stamp.nanoseconds() <= 0 ||
          !finiteTwist(msg->twist.twist) ||
          (odom_ && stamp < rclcpp::Time(odom_->header.stamp, clock_->get_clock_type())))
        {
          odom_.reset();
          fail("Invalid or out-of-order odometry");
          return;
        }
        odom_ = *msg;
        odom_received_ = TaskManager::Clock::now();
      });
    timer_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
      [this]() {tick();});
    setState(ExecutionState::IDLE, "");
    // Do not let absent local TF block goal/cancel callbacks or planning.
    activation_ = std::thread([this]() {
      try {
        while (!stopping_ && rclcpp::ok()) {
          std::string error;
          if (costmap_->getTfBuffer()->canTransform(costmap_->getGlobalFrameID(),
              costmap_->getBaseFrameID(), tf2::TimePointZero, &error))
          {
            active_ = costmap_->activate().label() == "active";
            if (!active_) {RCLCPP_ERROR(logger_, "Local costmap activation failed");}
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(logger_, "Local costmap activation failed: %s", exception.what());
      }
    });
  }

  ~Impl()
  {
    timer_.reset();
    publishZero();
    stopping_ = true;
    if (activation_.joinable()) {activation_.join();}
  }

  void publishZero() {cmd_pub_->publish(geometry_msgs::msg::Twist{});}

  void setState(ExecutionState state, const std::string & reason)
  {
    state_ = state;
    std_msgs::msg::String msg;
    msg.data = stateName(state);
    state_pub_->publish(msg);
    msg.data = reason;
    error_pub_->publish(msg);
  }

  void stop(const std::string & reason)
  {
    path_.reset();
    publishZero();  // Stop first, before potentially expensive optimizer reset.
    controller_.reset();
    setState(ExecutionState::STOPPED, reason);
  }

  void fail(const std::string & reason)
  {
    const bool had_path = path_.has_value();
    path_.reset();
    publishZero();
    if (had_path) {
      controller_.reset();
      setState(ExecutionState::FAILED, reason);
      RCLCPP_WARN(logger_, "Path execution stopped: %s", reason.c_str());
      failure_(identity_, reason);
    }
  }

  bool acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity)
  {
    // An obsolete handoff must neither replace the plan nor stop its newer executor.
    if (!confirmation_(identity)) {
      RCLCPP_WARN(logger_, "Ignoring unconfirmed path: goal_id=%lu plan_generation=%lu",
        static_cast<unsigned long>(identity.goal_id),
        static_cast<unsigned long>(identity.plan_generation));
      return false;
    }
    stop("Replacing execution path");
    identity_ = identity;
    try {
      if (path.poses.empty() || path.header.frame_id.empty()) {
        throw std::invalid_argument("Cannot execute an empty path or frame");
      }
      for (const auto & pose : path.poses) {
        if (!finitePose(pose.pose) || (!pose.header.frame_id.empty() &&
          pose.header.frame_id != path.header.frame_id))
        {
          throw std::invalid_argument("Invalid pose or inconsistent frame in execution path");
        }
      }
      controller_.setPlan(path);
      path_ = path;
      setState(ExecutionState::RUNNING, "");
      return true;
    } catch (const std::exception & exception) {
      // Acceptance failure must also invalidate the TaskManager product.
      publishZero();
      setState(ExecutionState::FAILED, exception.what());
      failure_(identity_, exception.what());
      return false;
    }
  }

  void checkStamp(const builtin_interfaces::msg::Time & stamp, double timeout,
    const std::string & description, bool allow_zero = false) const
  {
    const rclcpp::Time time(stamp, clock_->get_clock_type());
    if (allow_zero && time.nanoseconds() == 0) {return;}  // Static TF is timeless.
    const double age = (clock_->now() - time).seconds();
    if (time.nanoseconds() <= 0 || age > timeout || age < -0.1) {
      throw std::runtime_error(description + " timestamp is stale or invalid");
    }
  }

  geometry_msgs::msg::Twist readSpeed()
  {
    if (!odom_ || std::chrono::duration<double>(TaskManager::Clock::now() -
        odom_received_).count() > odom_timeout_)
    {
      throw std::runtime_error("Odometry missing or receive timeout");
    }
    checkStamp(odom_->header.stamp, odom_timeout_, "Odometry");
    if (odom_->child_frame_id == costmap_->getBaseFrameID()) {return odom_->twist.twist;}
    const auto transform = costmap_->getTfBuffer()->lookupTransform(
      costmap_->getBaseFrameID(), odom_->child_frame_id, odom_->header.stamp,
      rclcpp::Duration::from_seconds(0.0));
    return transformRobotVelocity(odom_->twist.twist, transform);
  }

  void tick()
  {
    if (!path_) {publishZero(); return;}
    if (!confirmation_(identity_)) {stop("TaskManager revoked execution path"); return;}
    try {
      if (!active_ || costmap_->get_current_state().label() != "active" ||
        !global_ready_() || !costmap_->isCurrent()) {
        throw std::runtime_error("Global or local costmap unavailable or not current");
      }
      geometry_msgs::msg::PoseStamped pose;
      if (!costmap_->getRobotPose(pose)) {throw std::runtime_error("Robot pose TF unavailable");}
      checkStamp(pose.header.stamp, tf_timeout_, "Robot TF", true);
      // Check the global path's frame connection too (map -> local odom).
      const auto path_tf = costmap_->getTfBuffer()->lookupTransform(
        costmap_->getGlobalFrameID(), path_->header.frame_id, tf2::TimePointZero);
      checkStamp(path_tf.header.stamp, tf_timeout_, "Path TF", true);
      const auto speed = readSpeed();
      auto goal = path_->poses.back();
      goal.header.frame_id = path_->header.frame_id;
      goal.header.stamp = builtin_interfaces::msg::Time{};
      goal = costmap_->getTfBuffer()->transform(goal, costmap_->getGlobalFrameID());
      if (!finitePose(pose.pose) || !finitePose(goal.pose)) {
        throw std::runtime_error("Robot or goal TF pose is invalid");
      }
      const double distance = std::hypot(pose.pose.position.x - goal.pose.position.x,
        pose.pose.position.y - goal.pose.position.y);
      const double yaw_error = std::remainder(
        tf2::getYaw(pose.pose.orientation) - tf2::getYaw(goal.pose.orientation), 2.0 * M_PI);
      if (distance <= tolerances_.xy && std::abs(yaw_error) <= tolerances_.yaw) {
        publishZero();
        if (std::hypot(speed.linear.x, speed.linear.y) <= tolerances_.stopped_speed &&
          std::abs(speed.angular.z) <= tolerances_.stopped_speed)
        {
          path_.reset();
          controller_.reset();
          setState(ExecutionState::SUCCEEDED, "");
          success_(identity_);
        }
        return;
      }
      auto * grid = costmap_->getCostmap();
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
        unsigned int mx, my;
        if (grid->getSizeInCellsX() < 3 || grid->getSizeInCellsY() < 3 ||
          !grid->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
        {
          throw std::runtime_error("Robot outside usable local costmap");
        }
      }
      const auto began = TaskManager::Clock::now();
      const auto command = controller_.computeVelocityCommands(pose, speed, &tolerances_);
      if (std::chrono::duration<double>(TaskManager::Clock::now() - began).count() >
        control_timeout_ || !finiteTwist(command.twist))
      {
        throw std::runtime_error("MPPI command invalid or computation deadline exceeded");
      }
      // Inputs can expire during a long computation. Recheck before publishing.
      checkStamp(pose.header.stamp, tf_timeout_, "Robot TF", true);
      checkStamp(path_tf.header.stamp, tf_timeout_, "Path TF", true);
      readSpeed();
      if (costmap_->get_current_state().label() != "active" ||
        !global_ready_() || !costmap_->isCurrent()) {
        throw std::runtime_error("Costmap invalidated during MPPI computation");
      }
      // Report validated controller feedback in the confirmed global path frame.
      // The monitor callback can synchronously revoke this path through TaskManager.
      tf2::Transform local_from_path;
      tf2::fromMsg(path_tf.transform, local_from_path);
      auto path_from_local = path_tf;
      path_from_local.header.frame_id = path_->header.frame_id;
      path_from_local.child_frame_id = costmap_->getGlobalFrameID();
      path_from_local.transform = tf2::toMsg(local_from_path.inverse());
      geometry_msgs::msg::PoseStamped route_pose;
      tf2::doTransform(pose, route_pose, path_from_local);
      observation_(identity_, route_pose, command.twist);
      if (!path_ || !confirmation_(identity_)) {publishZero(); return;}
      cmd_pub_->publish(command.twist);
    } catch (const std::exception & exception) {
      fail(exception.what());
    }
  }

  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::function<bool()> global_ready_;
  Failure failure_;
  Success success_;
  Confirmation confirmation_;
  Observation observation_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double frequency_, odom_timeout_, tf_timeout_, control_timeout_;
  GoalTolerances tolerances_;
  mppi_controller::MPPIController controller_;
  std::optional<nav_msgs::msg::Path> path_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  TaskManager::Clock::time_point odom_received_{};
  PlanIdentity identity_;
  ExecutionState state_{ExecutionState::IDLE};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, error_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> stopping_{false}, active_{false};
  std::thread activation_;
};

PathExecutor::PathExecutor(const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap,
  std::function<bool()> global_map_ready, Failure failure, Success success,
  Confirmation confirmation, Observation observation)
: impl_(std::make_unique<Impl>(node, std::move(local_costmap), std::move(global_map_ready),
    std::move(failure), std::move(success), std::move(confirmation), std::move(observation))) {}
PathExecutor::~PathExecutor() = default;
bool PathExecutor::acceptPath(const nav_msgs::msg::Path & path, PlanIdentity identity)
{return impl_->acceptPath(path, identity);}
void PathExecutor::stop(const std::string & reason) {impl_->stop(reason);}
ExecutionState PathExecutor::state() const {return impl_->state_;}
}  // namespace nav_executor
