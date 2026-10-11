#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "nav_executor/path_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_thread.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/string.hpp"
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
  ManagedCostmap() : Costmap2DROS("global_costmap") {}
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
    if (activated_) {
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
    tasks_(declare_parameter("goal_tolerance", 0.02), declare_parameter("retry_delay", 1.0))
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
          }
        } catch (const std::exception & exception) {
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

  ~NavExecutor() override
  {
    tick_.reset();
    worker_.reset();
    planner_executor_.remove_node(planner_node_);
    planner_.reset();
  }

private:
  void publishTask()
  {
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
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
  std::unique_ptr<nav2_util::NodeThread> costmap_thread;
  // ROS callback groups retain weak subscription control blocks until the base
  // node is destroyed. Keep their plugin code loaded beyond Costmap2DROS's
  // member ClassLoader destruction, including optimized builds.
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> plugin_libraries(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  try {
    costmap = std::make_shared<nav_executor::ManagedCostmap>();
    costmap_thread = std::make_unique<nav2_util::NodeThread>(costmap);
    if (costmap->configure().label() != "inactive") {
      throw std::runtime_error("Global costmap configuration failed");
    }
    for (const auto & key : {"plugins", "filters"}) {
      const auto names = costmap->get_parameter(key).as_string_array();
      for (const auto & name : names) {
        plugin_libraries.loadLibraryForClass(
          costmap->get_parameter(name + ".plugin").as_string());
      }
    }
    auto node = std::make_shared<nav_executor::NavExecutor>(costmap);
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
  if (costmap) {
    try {
      costmap->releaseResources();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(rclcpp::get_logger("nav_executor"), "Costmap cleanup failed: %s", exception.what());
      exit_code = 1;
    }
  }
  costmap_thread.reset();
  costmap.reset();
  rclcpp::shutdown();
  return exit_code;
}
