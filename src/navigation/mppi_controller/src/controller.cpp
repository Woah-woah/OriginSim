// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include "mppi_controller/controller.hpp"
#include "mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace mppi_controller
{

MPPIController::~MPPIController()
{
  if (configured_) {
    shutdown();
  }
}

void MPPIController::initialize(
  const rclcpp::Node::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  map_server::Costmap2D * costmap, const mppi::CostmapConfig & costmap_config)
{
  if (configured_) {
    throw std::logic_error("MPPIController is already configured");
  }
  if (parent.expired() || !tf || !costmap) {
    throw std::invalid_argument("MPPIController requires a node, TF and Costmap2D");
  }
  parent_ = parent;
  costmap_ = costmap;
  costmap_config_ = costmap_config;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_, parameters_handler_.get(), costmap_config_);
  path_handler_.initialize(parent_, name_, costmap_, tf_buffer_, parameters_handler_.get(),
      costmap_config_);
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_config_.global_frame, parameters_handler_.get());

  parameters_handler_->start();
  configured_ = true;
  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::shutdown()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  configured_ = false;
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  if (!configured_) {throw std::logic_error("MPPIController is not configured");}
  std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
  optimizer_.reset(false /*Don't reset zone-based speed limits between requests*/);
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  mppi::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  if (!configured_) {
    throw std::logic_error("Initialize MPPIController before computing commands");
  }
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  map_server::Costmap2D * costmap = costmap_;
  std::unique_lock<map_server::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan), cmd.header.stamp);
  }

  return cmd;
}

void MPPIController::visualize(
  nav_msgs::msg::Path transformed_plan,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  // trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp);
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!configured_) {
    throw std::logic_error("Initialize MPPIController before setting a plan");
  }
  if (path.poses.empty() || path.header.frame_id.empty()) {
    throw mppi::InvalidPath("Plan must contain poses and a frame_id");
  }
  std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (!configured_) {throw std::logic_error("MPPIController is not configured");}
  std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace mppi_controller
