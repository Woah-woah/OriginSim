#pragma once
#include "map_server/derived_maps.hpp"
namespace map_server
{
// Circle vs cell square, including tangent coverage; no ROI clipping.
void rasterizeObstacles(BinaryGrid &grid, const std::vector<ObstacleCircle> &targets);
void overlayConservative(BinaryGrid &destination, const BinaryGrid &source);
} // namespace map_server
