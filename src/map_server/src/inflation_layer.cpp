#include "map_server/costmap_layers.hpp"
#include "map_server/distance_field.hpp"

namespace map_server
{
void inflate(Costmap2D & map, const InflationOptions & options, const DistanceField * supplied)
{
  if (!std::isfinite(options.radius) || options.radius < 0 ||
    !std::isfinite(options.inscribed_radius) || options.inscribed_radius < 0 ||
    !std::isfinite(options.cost_scaling_factor) || options.cost_scaling_factor < 0)
  {throw std::invalid_argument("Invalid inflation parameters");}
  if (options.radius == 0) {return;}
  const int width = map.getSizeInCellsX(), height = map.getSizeInCellsY();
  // 复用融合快照的 EDT；未知种子策略不同时才单独计算兼容膨胀距离。
  DistanceField owned;
  if (!supplied) {owned = computeDistanceField(map, options.inflate_around_unknown); supplied = &owned;}
  if (!supplied->matches(map)) {throw std::invalid_argument("Inflation EDT geometry mismatch");}
  const bool has_seed = std::any_of(supplied->obstacles.begin(), supplied->obstacles.end(),
    [&](uint8_t cost) {return cost == LETHAL_OBSTACLE ||
      (options.inflate_around_unknown && cost == NO_INFORMATION);});
  if (!has_seed) {return;}
  // Nav2 rounds the inflation radius up to a whole cell.
  const double cell_radius = std::ceil(options.radius / map.getResolution());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double approximate = supplied->distances[static_cast<size_t>(y) * width + x] / map.getResolution();
      // Lattice squared distances are integers; restore double precision before
      // thresholding and quantizing to match the uint8 inflation formula.
      const double cells = std::sqrt(std::round(approximate * approximate));
      if (cells > cell_radius) {continue;}
      const double distance = cells * map.getResolution();
      const uint8_t cost = cells == 0 ? LETHAL_OBSTACLE :
        distance <= options.inscribed_radius ? INSCRIBED_INFLATED_OBSTACLE :
        static_cast<uint8_t>(252.0 * std::exp(
          -options.cost_scaling_factor * (distance - options.inscribed_radius)));
      const auto previous = map.getCost(x, y);
      if (previous == NO_INFORMATION) {
        if ((options.inflate_unknown && cost > FREE_SPACE) ||
          (!options.inflate_unknown && cost >= INSCRIBED_INFLATED_OBSTACLE))
        {map.setCost(x, y, cost);}
      } else {map.setCost(x, y, std::max(previous, cost));}
    }
  }
}

}  // namespace map_server
