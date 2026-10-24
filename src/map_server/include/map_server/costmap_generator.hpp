#pragma once
#include "map_server/derived_maps.hpp"
namespace map_server
{
struct CostmapOptions
{
  double robot_radius{0.15}, safety_margin{0.0}, inflation_radius{0.55}, cost_scaling_factor{5.0};
  double grid_margin_factor{0.7071067811865476};
  void validate() const;
};
void generateCosts(DerivedMaps &maps, const CostmapOptions &options);
} // namespace map_server
