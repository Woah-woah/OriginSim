#pragma once
#include "map_server/grid.hpp"
#include <optional>
#include <string>
namespace map_server
{
struct StaticMap
{
  GridGeometry geometry;
  BinaryGrid occupancy;
  RegionGrid region_id;
  std::optional<Grid<float>> height, depth;
};
// Every supplied semantic layer has its own YAML (image/resolution/origin).
// Region YAML additionally declares a complete labels list; image is uint16.
struct StaticMapFiles
{
  std::string occupancy_yaml, region_yaml, height_yaml, depth_yaml, static_pcd;
};
StaticMap loadStaticMapFiles(const StaticMapFiles &files);
} // namespace map_server
