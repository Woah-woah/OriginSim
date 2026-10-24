#include "map_server/distance_field.hpp"
#include <algorithm>
namespace map_server
{
bool sampleDistance(const DerivedMaps &maps, double x, double y, DistanceSample &value)
{
  const auto &g = maps.geometry;
  const auto width = g.width, height = g.height;
  const auto resolution = g.resolution, origin_x = g.origin_x, origin_y = g.origin_y;
  const auto &obstacles = maps.fused_obstacles.data;
  const auto &distances = maps.distance_field.data;
  value = {};
  const double ex = origin_x + width * resolution, ey = origin_y + height * resolution;
  if (!std::isfinite(x) || !std::isfinite(y) || x < origin_x || y < origin_y || x >= ex ||
      y >= ey || !width || !height)
  {
    return false;
  }
  const unsigned int cell_x = (x - origin_x) / resolution, cell_y = (y - origin_y) / resolution;
  const auto occupancy = obstacles[static_cast<size_t>(cell_y) * width + cell_x];
  if (occupancy)
  {
    return true;
  }
  // 栅格中心双线性插值；边缘半格保持常值，梯度与同一插值严格一致。
  const double ux = (x - origin_x) / resolution - 0.5;
  const double uy = (y - origin_y) / resolution - 0.5;
  const double cx = std::clamp(ux, 0.0, static_cast<double>(width - 1));
  const double cy = std::clamp(uy, 0.0, static_cast<double>(height - 1));
  const unsigned int x0 = std::floor(cx), y0 = std::floor(cy);
  const unsigned int x1 = std::min(x0 + 1, width - 1), y1 = std::min(y0 + 1, height - 1);
  const double tx = cx - x0, ty = cy - y0;
  const auto at = [&](unsigned int ix, unsigned int iy)
  { return distances[static_cast<size_t>(iy) * width + ix]; };
  const double a = at(x0, y0), b = at(x1, y0), c = at(x0, y1), d = at(x1, y1);
  value.distance = (1 - ty) * ((1 - tx) * a + tx * b) + ty * ((1 - tx) * c + tx * d);
  value.dx = ux >= 0 && ux < width - 1 ? ((1 - ty) * (b - a) + ty * (d - c)) / resolution : 0;
  value.dy = uy >= 0 && uy < height - 1 ? ((1 - tx) * (c - a) + tx * (d - b)) / resolution : 0;
  // 发布窗口外并非已知自由；边界距离提供保守软代价，硬约束另行检查车体。
  const double boundary[] = {x - origin_x, ex - x, y - origin_y, ey - y};
  const auto nearest = std::min_element(std::begin(boundary), std::end(boundary));
  if (*nearest < value.distance)
  {
    value.distance = *nearest;
    const auto side = nearest - boundary;
    value.dx = side == 0 ? 1 : side == 1 ? -1 : 0;
    value.dy = side == 2 ? 1 : side == 3 ? -1 : 0;
  }
  return std::isfinite(value.distance) && value.distance >= 0 && std::isfinite(value.dx) &&
         std::isfinite(value.dy);
}

} // namespace map_server
