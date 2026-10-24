#pragma once
#include "map_server/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace map_server
{
struct MapSettings
{
  std::string global_frame, robot_base_frame, footprint_frame;
  Footprint footprint;
  double robot_radius, inflation_radius, cost_scaling_factor;
  bool track_unknown;
};
MapSettings declareMapSettings(rclcpp::Node &node);
} // namespace map_server
