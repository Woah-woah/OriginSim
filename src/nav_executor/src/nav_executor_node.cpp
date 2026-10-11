#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "nav_executor/path_planner.hpp"
#include "nav_executor/path_executor.hpp"
#include "nav_executor/route_monitor.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_thread.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "topology_global_planner/topology_global_planner.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;
void requestShutdown(int) {shutdown_requested = 1;}
}  // namespace

namespace nav_executor
{
// The existing planner silently disables topology on load failure. Validate the
// schema it consumes here so bad configuration cannot silently change routing.
void validateTopology(const std::string & filename)
{
  const auto root = YAML::LoadFile(filename);
  if (!root["regions"].IsSequence() || root["regions"].size() == 0 ||
    !root["connectors"].IsSequence() || root["connectors"].size() == 0)
  {
    throw std::invalid_argument("Topology requires nonempty regions and connectors sequences");
  }
  const auto point = [](const YAML::Node & value) {
      const double x = value["x"].as<double>();
      const double y = value["y"].as<double>();
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("Topology coordinates must be finite");
      }
      return std::pair<double, double>{x, y};
    };
  std::unordered_set<std::string> regions;
  for (const auto & region : root["regions"]) {
    const auto id = region["id"].as<std::string>();
    if (id.empty() || !regions.insert(id).second ||
      !region["polygon"].IsSequence() || region["polygon"].size() < 3)
    {
      throw std::invalid_argument("Invalid topology region ID or polygon");
    }
    if (region["name"]) {region["name"].as<std::string>();}
    for (const auto & vertex : region["polygon"]) {point(vertex);}
  }
  std::unordered_set<std::string> connectors;
  for (const auto & connector : root["connectors"]) {
    const auto id = connector["id"].as<std::string>();
    const auto mode = connector["mode"].as<std::string>();
    const double cost = connector["cost"] ? connector["cost"].as<double>() : 1.0;
    if (id.empty() || !connectors.insert(id).second ||
      !regions.count(connector["from"].as<std::string>()) ||
      !regions.count(connector["to"].as<std::string>()) ||
      (mode != "two_way" && mode != "one_way") || !std::isfinite(cost) || cost < 0.0 ||
      point(connector["portal"]["start"]) == point(connector["portal"]["end"]))
    {
      throw std::invalid_argument("Invalid topology connector: " + id);
    }
    connector["action"].as<std::string>();
  }
}

// The executable owns lifecycle transitions. Stop the planning worker before cleanup.
class ManagedCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  ManagedCostmap(const std::string & name, bool use_sim_time)
  : Costmap2DROS(name, use_sim_time) {}
  void on_rcl_preshutdown() override {}
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    configured_ = true;
    return Costmap2DROS::on_configure(state);
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = Costmap2DROS::on_activate(state);
    activated_ = result == nav2_util::CallbackReturn::SUCCESS;
    return result;
  }
  void releaseResources()
  {
    const auto state = get_current_state();
    if (activated_ && state.label() == "active") {
      deactivate();
      activated_ = false;
    }
    if (configured_) {
      if (get_current_state().label() == "inactive") {
        cleanup();
      } else {
        Costmap2DROS::on_cleanup(state);
      }
      configured_ = false;
    }
  }

private:
  bool configured_{false};
  bool activated_{false};
};

class NavExecutor : public rclcpp::Node
{
public:
  explicit NavExecutor(std::shared_ptr<ManagedCostmap> costmap)
  : Node("nav_executor"), costmap_(std::move(costmap)),
    tasks_(declare_parameter("goal_tolerance", 0.02), declare_parameter("retry_delay", 1.0),
      declare_parameter("route_monitor.replan_cooldown", 2.0))
  {
    planning_timeout_ = declare_parameter("planning_timeout", 5.0);
    const double tick_frequency = declare_parameter("tick_frequency", 20.0);
    if (!std::isfinite(planning_timeout_) || planning_timeout_ <= 0.0 ||
      !std::isfinite(tick_frequency) || tick_frequency <= 0.0 || tick_frequency > 1000.0)
    {
      throw std::invalid_argument("planning_timeout > 0 and 0 < tick_frequency <= 1000 required");
    }
    global_frame_ = costmap_->getGlobalFrameID();
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", qos);
    state_pub_ = create_publisher<std_msgs::msg::String>("nav_executor/state", qos);
    error_pub_ = create_publisher<std_msgs::msg::String>("nav_executor/error", qos);
    reason_pub_ = create_publisher<std_msgs::msg::String>("nav_executor/replan_reason", qos);
    count_pub_ = create_publisher<std_msgs::msg::UInt64>("nav_executor/replan_count", qos);
    goal_id_pub_ = create_publisher<std_msgs::msg::UInt64>("nav_executor/goal_id", qos);
    generation_pub_ = create_publisher<std_msgs::msg::UInt64>("nav_executor/plan_generation", qos);
    RouteMonitorConfig monitor_config;
    monitor_config.enabled = declare_parameter("route_monitor.enabled", true);
    monitor_config.deviation_distance = declare_parameter("route_monitor.deviation_distance", 0.5);
    monitor_config.deviation_duration = declare_parameter("route_monitor.deviation_duration", 1.0);
    monitor_config.stall_duration = declare_parameter("route_monitor.stall_duration", 3.0);
    monitor_config.minimum_command_speed =
      declare_parameter("route_monitor.minimum_command_speed", 0.1);
    monitor_config.minimum_progress = declare_parameter("route_monitor.minimum_progress", 0.1);
    monitor_config.maximum_sample_gap = declare_parameter("route_monitor.maximum_sample_gap", 0.25);
    monitor_ = std::make_unique<RouteMonitor>(monitor_config);

    // Separate node so the planner's existing mutable connector callbacks can be
    // serviced exclusively on the planning thread, between planning calls.
    planner_node_ = std::make_shared<rclcpp::Node>("topology_planner");
    const bool use_topology = planner_node_->declare_parameter("TopoPlanner.use_topology", true);
    const auto yaml = planner_node_->declare_parameter("TopoPlanner.topology_yaml", std::string{});
    if (use_topology && yaml.empty()) {
      throw std::invalid_argument("TopoPlanner.topology_yaml is required when topology is enabled");
    }
    if (use_topology) {validateTopology(yaml);}
    planner_ = std::make_unique<topology_global_planner::TopologyGlobalPlanner>();
    planner_->initialize(planner_node_, costmap_->getTfBuffer(), &snapshot_,
      global_frame_, costmap_->getBaseFrameID());
    planner_executor_.add_node(planner_node_);
    worker_ = std::make_unique<PathPlanner>(
      [this](const PlanningRequest & request, PathPlanner::CancelChecker cancelled) {
        if (!costmap_->isCurrent()) {throw std::runtime_error("Global costmap is not current");}
        geometry_msgs::msg::PoseStamped start;
        if (!costmap_->getRobotPose(start)) {throw std::runtime_error("Robot pose TF unavailable");}
        // One snapshot for ALL topology segments. Live map updates keep running.
        auto * live = costmap_->getCostmap();
        {
          std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
          snapshot_ = *live;
        }
        if (snapshot_.getSizeInCellsX() < 3 || snapshot_.getSizeInCellsY() < 3) {
          throw std::runtime_error("Global costmap has no usable map");
        }
        const auto deadline = TaskManager::Clock::now() +
          std::chrono::duration<double>(planning_timeout_);
        auto timed_cancel = [cancelled, deadline]() {
            return cancelled() || !rclcpp::ok() || TaskManager::Clock::now() >= deadline;
          };
        auto path = planner_->plan(start, request.goal, timed_cancel);
        if (TaskManager::Clock::now() >= deadline) {
          throw std::runtime_error("Global planning timed out");
        }
        return path;
      },
      [this]() {
        if (!rclcpp::ok()) {return;}
        try {
          if (!activation_attempted_) {
            // Avoid blocking activation on absent localization, including when
            // the simulation clock is paused. Keep cancellation/shutdown live.
            if (!costmap_->getTfBuffer()->canTransform(
                global_frame_, costmap_->getBaseFrameID(), tf2::TimePointZero))
            {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for TF from %s to %s", costmap_->getBaseFrameID().c_str(),
                global_frame_.c_str());
              return;
            }
            activation_attempted_ = true;
            const auto & state = costmap_->activate();
            if (state.label() != "active") {
              RCLCPP_ERROR(get_logger(), "Global costmap activation failed");
            }
          }
          if (rclcpp::ok()) {planner_executor_.spin_some();}
          ready_ = costmap_->get_current_state().label() == "active" && costmap_->isCurrent();
        } catch (const std::exception & exception) {
          ready_ = false;
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "Planner callback or costmap activation failed: %s", exception.what());
        }
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS(10), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (executor_) {executor_->stop("Goal update");}
        try {
          auto goal = *msg;
          if (goal.header.frame_id.empty()) {throw std::invalid_argument("Goal frame_id is empty");}
          const auto & p = goal.pose.position;
          const auto & q = goal.pose.orientation;
          const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(norm) || norm < 1e-6)
          {
            throw std::invalid_argument("Goal has invalid position or quaternion");
          }
          goal.pose.orientation.x /= norm;
          goal.pose.orientation.y /= norm;
          goal.pose.orientation.z /= norm;
          goal.pose.orientation.w /= norm;
          if (goal.header.frame_id != global_frame_) {
            goal = costmap_->getTfBuffer()->transform(goal, global_frame_, tf2::durationFromSec(0.1));
          }
          if (tasks_.setGoal(goal)) {
            worker_->invalidate(tasks_.generation());
            publishTask();
          } else if (executor_ && tasks_.state() == TaskState::PATH_READY) {
            executor_->acceptPath(tasks_.path(), tasks_.identity());
          }
        } catch (const std::exception & exception) {
          tasks_.cancel();
          worker_->invalidate(tasks_.generation());
          publishTask();
          RCLCPP_WARN(get_logger(), "Rejected goal: %s", exception.what());
          std_msgs::msg::String error;
          error.data = exception.what();
          error_pub_->publish(error);
        }
      });
    cancel_srv_ = create_service<std_srvs::srv::Trigger>("nav_executor/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        tasks_.cancel();
        if (executor_) {executor_->stop("Task cancelled");}
        worker_->invalidate(tasks_.generation());
        publishTask();
        response->success = true;
        response->message = "Task cancelled";
      });
    replan_srv_ = create_service<std_srvs::srv::Trigger>("nav_executor/replan",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = tasks_.replan();
        response->message = response->success ? "Replan requested" : "No active goal";
        if (response->success) {
          if (executor_) {executor_->stop("Replan requested");}
          worker_->invalidate(tasks_.generation());
          publishTask();
        }
      });
    tick_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_frequency), [this]() {
      const auto now = TaskManager::Clock::now();
      if (auto result = worker_->takeResult()) {
        if (tasks_.accept(std::move(*result), now)) {
          if (tasks_.state() == TaskState::FAILED) {
            RCLCPP_WARN(get_logger(), "Planning failed: %s", tasks_.error().c_str());
          }
          if (executor_ && tasks_.state() == TaskState::PATH_READY) {
            executor_->acceptPath(tasks_.path(), tasks_.identity());
          }
          publishTask();
        }
      }
      if (auto request = tasks_.nextRequest(ready_, now)) {
        worker_->submit(std::move(*request));
        publishTask();
      }
    });
    publishTask();
  }

  void initializeExecutor(const std::shared_ptr<ManagedCostmap> & local_costmap)
  {
    executor_ = std::make_unique<PathExecutor>(shared_from_this(), local_costmap,
      [this]() {
        if (!ready_ || costmap_->get_current_state().label() != "active") {return false;}
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getCostmap()->getMutex());
        return costmap_->isCurrent();
      },
      [this](PlanIdentity identity, const std::string & error) {
        if (tasks_.executionFailed(identity, error, TaskManager::Clock::now())) {
          worker_->invalidate(tasks_.generation());
          publishTask();
        }
      },
      [this](PlanIdentity identity) {
        if (tasks_.executionSucceeded(identity)) {publishTask();}
      },
      [this](PlanIdentity identity) {return tasks_.isPathConfirmed(identity);},
      [this](PlanIdentity identity, const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & command) {
        if (!tasks_.isPathConfirmed(identity)) {return;}
        const auto now = TaskManager::Clock::now();
        const auto event = monitor_->observe(tasks_.path(), identity, pose, command, now);
        if (event && tasks_.routeInvalid(*event, now)) {
          executor_->stop(TaskManager::reasonName(event->reason));
          worker_->invalidate(tasks_.generation());
          RCLCPP_WARN(get_logger(),
            "Route replan: reason=%s goal_id=%lu old_generation=%lu new_generation=%lu "
            "count=%lu distance=%.3f progress=%.3f duration=%.2f; stopped until confirmed plan",
            TaskManager::reasonName(event->reason), static_cast<unsigned long>(identity.goal_id),
            static_cast<unsigned long>(identity.plan_generation),
            static_cast<unsigned long>(tasks_.generation()),
            static_cast<unsigned long>(tasks_.replanCount()),
            event->path_distance, event->progress, event->duration);
          publishTask();
        }
      });
  }

  ~NavExecutor() override
  {
    tick_.reset();
    executor_.reset();
    worker_.reset();
    planner_executor_.remove_node(planner_node_);
    planner_.reset();
  }

private:
  void publishTask()
  {
    if (monitor_ && tasks_.state() != TaskState::PATH_READY) {monitor_->reset();}
    if (executor_ && executor_->state() == ExecutionState::RUNNING &&
      tasks_.state() != TaskState::PATH_READY &&
      tasks_.state() != TaskState::SUCCEEDED)
    {
      executor_->stop(TaskManager::stateName(tasks_.state()));
    }
    auto path = tasks_.path();
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    path_pub_->publish(path);  // Empty paths revoke obsolete latched paths.
    std_msgs::msg::String state;
    state.data = TaskManager::stateName(tasks_.state());
    state_pub_->publish(state);
    std_msgs::msg::String error;
    error.data = tasks_.error();
    error_pub_->publish(error);
    state.data = TaskManager::reasonName(tasks_.lastReplanReason());
    reason_pub_->publish(state);
    std_msgs::msg::UInt64 number;
    number.data = tasks_.replanCount();
    count_pub_->publish(number);
    number.data = tasks_.identity().goal_id;
    goal_id_pub_->publish(number);
    number.data = tasks_.generation();
    generation_pub_->publish(number);
    RCLCPP_INFO(get_logger(),
      "Task state=%s goal_id=%lu plan_generation=%lu route_replan_count=%lu reason=%s error=%s",
      TaskManager::stateName(tasks_.state()), static_cast<unsigned long>(tasks_.identity().goal_id),
      static_cast<unsigned long>(tasks_.generation()),
      static_cast<unsigned long>(tasks_.replanCount()),
      TaskManager::reasonName(tasks_.lastReplanReason()), tasks_.error().c_str());
  }

  std::shared_ptr<ManagedCostmap> costmap_;
  TaskManager tasks_;
  std::string global_frame_;
  double planning_timeout_;
  std::atomic<bool> ready_{false};
  bool activation_attempted_{false};  // Planning thread only.
  nav2_costmap_2d::Costmap2D snapshot_;
  rclcpp::Node::SharedPtr planner_node_;
  rclcpp::executors::SingleThreadedExecutor planner_executor_;
  std::unique_ptr<topology_global_planner::TopologyGlobalPlanner> planner_;
  std::unique_ptr<PathPlanner> worker_;
  std::unique_ptr<PathExecutor> executor_;
  std::unique_ptr<RouteMonitor> monitor_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr count_pub_, goal_id_pub_, generation_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  rclcpp::TimerBase::SharedPtr tick_;
};
}  // namespace nav_executor

int main(int argc, char ** argv)
{
  // Keep ROS communications alive while the worker and lifecycle resources
  // are stopped. The default signal handler shuts the context down too early.
  rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);
  int exit_code = 0;
  std::shared_ptr<nav_executor::ManagedCostmap> costmap;
  std::shared_ptr<nav_executor::ManagedCostmap> local_costmap;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread;
  std::unique_ptr<nav2_util::NodeThread> local_costmap_thread;
  // ROS callback groups retain weak subscription control blocks until the base
  // node is destroyed. Keep their plugin code loaded beyond Costmap2DROS's
  // member ClassLoader destruction, including optimized builds.
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> plugin_libraries(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  try {
    // The named Costmap2DROS constructor adds a use_sim_time CLI override,
    // which takes precedence over YAML. Resolve the executor's configured
    // clock before creating either map and explicitly pass the same time mode.
    const bool use_sim_time = []() {
        const auto parameters = std::make_shared<rclcpp::Node>("nav_executor");
        return parameters->get_parameter("use_sim_time").as_bool();
      }();
    costmap = std::make_shared<nav_executor::ManagedCostmap>("global_costmap", use_sim_time);
    local_costmap = std::make_shared<nav_executor::ManagedCostmap>("local_costmap", use_sim_time);
    costmap_thread = std::make_unique<nav2_util::NodeThread>(costmap);
    local_costmap_thread = std::make_unique<nav2_util::NodeThread>(local_costmap);
    if (costmap->configure().label() != "inactive") {
      throw std::runtime_error("Global costmap configuration failed");
    }
    if (local_costmap->configure().label() != "inactive") {
      throw std::runtime_error("Local costmap configuration failed");
    }
    for (const auto & map : {costmap, local_costmap}) {
      for (const auto & key : {"plugins", "filters"}) {
        const auto names = map->get_parameter(key).as_string_array();
        for (const auto & name : names) {
          plugin_libraries.loadLibraryForClass(map->get_parameter(name + ".plugin").as_string());
        }
      }
    }
    auto node = std::make_shared<nav_executor::NavExecutor>(costmap);
    node->initializeExecutor(local_costmap);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !shutdown_requested) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("nav_executor"), "%s", exception.what());
    exit_code = 1;
  }
  // The worker has joined; finish lifecycle transitions before context shutdown.
  for (const auto & map : {local_costmap, costmap}) {
    if (!map) {continue;}
    try {
      map->releaseResources();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(rclcpp::get_logger("nav_executor"), "Costmap cleanup failed: %s", exception.what());
      exit_code = 1;
    }
  }
  local_costmap_thread.reset();
  local_costmap.reset();
  costmap_thread.reset();
  costmap.reset();
  rclcpp::shutdown();
  return exit_code;
}
