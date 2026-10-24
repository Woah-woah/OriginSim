#pragma once
#include "map_server/grid_geometry.hpp"
#include <vector>
namespace map_server
{
template <class T> struct Grid
{
  GridGeometry geometry;
  std::vector<T> data;
  void validate() const
  {
    geometry.validate();
    if (data.size() != geometry.size())
      throw std::invalid_argument("Grid data size mismatch");
  }
};
using BinaryGrid = Grid<uint8_t>;
using RegionId = uint16_t;
constexpr RegionId INVALID_REGION = 0;
using RegionGrid = Grid<RegionId>;
using DistanceGrid = Grid<float>;
using CostGrid = Grid<float>;
} // namespace map_server
