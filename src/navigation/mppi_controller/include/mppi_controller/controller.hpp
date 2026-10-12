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

#ifndef MPPI_CONTROLLER__CONTROLLER_HPP_
#define MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

#include "mppi_controller/tools/path_handler.hpp"
#include "mppi_controller/optimizer.hpp"
#include "mppi_controller/tools/trajectory_visualizer.hpp"
#include "mppi_controller/models/constraints.hpp"
#include "mppi_controller/tools/utils.hpp"

#include "mppi_controller/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Standalone MPPI controller using an externally owned Costmap2D
 */
class MPPIController
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;
  ~MPPIController();
  MPPIController(const MPPIController &) = delete;
  MPPIController & operator=(const MPPIController &) = delete;

  /**
    * @brief Initialize the standalone algorithm
    * @param parent WeakPtr to node
    * @param name Parameter namespace
    * @param tf TF buffer to use
    * @param costmap Costmap2D object of environment
    */
  void initialize(
    const rclcpp::Node::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    map_server::Costmap2D * costmap, const mppi::CostmapConfig & costmap_config = {});

  /**
    * @brief Release algorithm resources
    */
  void shutdown();

  /**
    * @brief Reset the controller state between tasks
    */
  void reset();

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    mppi::GoalChecker * goal_checker = nullptr);

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage);

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(
    nav_msgs::msg::Path transformed_plan,
    const builtin_interfaces::msg::Time & cmd_stamp);

  std::string name_;
  rclcpp::Node::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  map_server::Costmap2D * costmap_{nullptr};
  mppi::CostmapConfig costmap_config_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_{false};
  bool configured_{false};
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLER_HPP_
