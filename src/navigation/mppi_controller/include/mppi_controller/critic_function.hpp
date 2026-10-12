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

#ifndef MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "map_server/costmap_2d.hpp"
#include "mppi_controller/costmap_config.hpp"

#include "mppi_controller/tools/parameters_handler.hpp"
#include "mppi_controller/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap Costmap2D object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp::Node::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    map_server::Costmap2D * costmap,
    ParametersHandler * param_handler, const CostmapConfig & costmap_config = {})
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ = costmap;
    costmap_config_ = costmap_config;
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize() = 0;

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp::Node::WeakPtr parent_;
  map_server::Costmap2D * costmap_{nullptr};
  CostmapConfig costmap_config_;

  ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
