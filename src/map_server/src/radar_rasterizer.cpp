#include "map_server/radar_rasterizer.hpp"
#include <algorithm>
namespace map_server
{
void rasterizeObstacles(BinaryGrid &grid, const std::vector<ObstacleCircle> &targets)
{
  grid.validate();
  const auto &g = grid.geometry;
  for (const auto &t : targets)
  {
    if (!t.center.allFinite() || !std::isfinite(t.radius_m) || t.radius_m < 0)
      throw std::invalid_argument("Invalid radar circle");
    const double r = t.radius_m;
    const auto bound = [](double value, int limit)
    { return static_cast<int>(std::clamp(value, 0.0, static_cast<double>(limit))); };
    int x0 = bound(std::floor((t.center.x() - r - g.origin_x) / g.resolution), g.width);
    int y0 = bound(std::floor((t.center.y() - r - g.origin_y) / g.resolution), g.height);
    int x1 = bound(std::floor((t.center.x() + r - g.origin_x) / g.resolution) + 1, g.width);
    int y1 = bound(std::floor((t.center.y() + r - g.origin_y) / g.resolution) + 1, g.height);
    for (int y = y0; y < y1; ++y)
      for (int x = x0; x < x1; ++x)
      {
        const auto c = g.center(static_cast<size_t>(y) * g.width + x);
        double dx = std::max(std::abs(t.center.x() - c.x()) - g.resolution * 0.5, 0.0);
        double dy = std::max(std::abs(t.center.y() - c.y()) - g.resolution * 0.5, 0.0);
        if (dx * dx + dy * dy <= r * r)
          grid.data[static_cast<size_t>(y) * g.width + x] = 1;
      }
  }
}
void overlayConservative(BinaryGrid &out, const BinaryGrid &in)
{
  out.validate();
  in.validate();
  const auto &a = out.geometry;
  const auto &b = in.geometry;
  for (size_t i = 0; i < in.data.size(); ++i)
    if (in.data[i])
    {
      double lx = b.origin_x + (i % b.width) * b.resolution,
             ly = b.origin_y + (i / b.width) * b.resolution;
      const auto bound = [](double v, int n)
      { return static_cast<int>(std::clamp(v, 0.0, static_cast<double>(n))); };
      int x0 = bound(std::floor((lx - a.origin_x) / a.resolution), a.width);
      int y0 = bound(std::floor((ly - a.origin_y) / a.resolution), a.height);
      int x1 = bound(std::ceil((lx + b.resolution - a.origin_x) / a.resolution), a.width);
      int y1 = bound(std::ceil((ly + b.resolution - a.origin_y) / a.resolution), a.height);
      for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x)
          out.data[static_cast<size_t>(y) * a.width + x] = 1;
    }
}
} // namespace map_server
