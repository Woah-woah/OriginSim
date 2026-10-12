#include "map_server/costmap_layers.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace map_server
{
Costmap2D occupancyToCostmap(const nav_msgs::msg::OccupancyGrid & source,
  bool track_unknown, int threshold, bool trinary)
{
  const auto & info = source.info;
  const auto & q = info.origin.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!info.width || !info.height || source.header.frame_id.empty() ||
    source.data.size() != static_cast<size_t>(info.width) * info.height ||
    !std::isfinite(info.resolution) || info.resolution <= 0 ||
    !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
    !std::isfinite(norm) || std::abs(norm - 1) > 1e-6 ||
    std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || threshold < 1 || threshold > 100)
  {throw std::invalid_argument("Invalid OccupancyGrid geometry or lethal threshold");}
  for (auto value : source.data) {
    if (value < -1 || value > 100) {throw std::invalid_argument("Invalid occupancy probability");}
  }
  const double yaw = tf2::getYaw(q), c = std::cos(yaw), s = std::sin(yaw);
  const double ox = info.origin.position.x, oy = info.origin.position.y;
  const double width = info.width * info.resolution, height = info.height * info.resolution;
  double min_x = ox, max_x = ox, min_y = oy, max_y = oy;
  for (double x : {0.0, width}) {
    for (double y : {0.0, height}) {
      min_x = std::min(min_x, ox + c * x - s * y); max_x = std::max(max_x, ox + c * x - s * y);
      min_y = std::min(min_y, oy + s * x + c * y); max_y = std::max(max_y, oy + s * x + c * y);
    }
  }
  const double cells_x = std::ceil((max_x - min_x) / info.resolution - 1e-8);
  const double cells_y = std::ceil((max_y - min_y) / info.resolution - 1e-8);
  if (cells_x < 1 || cells_y < 1 || cells_x * cells_y > 100000000)
  {throw std::invalid_argument("Excessive rotated map geometry");}
  Costmap2D output(cells_x, cells_y, info.resolution, min_x, min_y,
    track_unknown ? NO_INFORMATION : FREE_SPACE);
  for (unsigned int y = 0; y < output.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < output.getSizeInCellsX(); ++x) {
      double wx, wy; output.mapToWorld(x, y, wx, wy);
      const double sx = (c * (wx - ox) + s * (wy - oy)) / info.resolution;
      const double sy = (-s * (wx - ox) + c * (wy - oy)) / info.resolution;
      if (sx < 0 || sy < 0 || sx >= info.width || sy >= info.height) {continue;}
      const int value = source.data[static_cast<size_t>(sy) * info.width + static_cast<size_t>(sx)];
      output.setCost(x, y, value == -1 ? (track_unknown ? NO_INFORMATION : FREE_SPACE) :
        value >= threshold ? LETHAL_OBSTACLE : trinary ? FREE_SPACE :
        static_cast<uint8_t>(254.0 * value / threshold));
    }
  }
  return output;
}

}  // namespace map_server
