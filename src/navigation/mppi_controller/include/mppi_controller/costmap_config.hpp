// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#ifndef MPPI_CONTROLLER__COSTMAP_CONFIG_HPP_
#define MPPI_CONTROLLER__COSTMAP_CONFIG_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "map_server/cost_values.hpp"
#include "map_server/footprint.hpp"

namespace mppi
{
// Geometry and cost interpretation accompanying the caller-owned raw grid.
// No ROS wrapper, layer pointers, lifecycle objects or ownership of the costmap.
struct CostmapConfig
{
  std::string global_frame{"odom"};
  std::string robot_base_frame{"base_link"};
  std::vector<geometry_msgs::msg::Point> footprint;
  double robot_radius{0.22};
  bool track_unknown{false};
  double inflation_radius{0.0};
  double inflation_cost_scaling_factor{0.0};

  double inscribedRadius() const
  {
    return footprint.empty() ? robot_radius :
           map_server::calculateMinAndMaxDistances(footprint).first;
  }

  double circumscribedRadius() const
  {
    return footprint.empty() ? robot_radius :
           map_server::calculateMinAndMaxDistances(footprint).second;
  }

  float circumscribedCost() const
  {
    const double radius = circumscribedRadius();
    if (inflation_radius < radius || inflation_cost_scaling_factor <= 0.0) {
      return -1.0f;  // No valid shortcut: check the full footprint instead.
    }
    if (radius <= inscribedRadius()) {
      return map_server::INSCRIBED_INFLATED_OBSTACLE;
    }
    return static_cast<unsigned char>(252.0 * std::exp(
             -inflation_cost_scaling_factor * (radius - inscribedRadius())));
  }
};
}  // namespace mppi
#endif  // MPPI_CONTROLLER__COSTMAP_CONFIG_HPP_
