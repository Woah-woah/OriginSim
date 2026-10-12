#include "map_server/costmap_layers.hpp"
#include <opencv2/imgproc.hpp>

namespace map_server
{
void clearFootprint(Costmap2D & map, double x, double y, double yaw, const Footprint & footprint)
{
  if (footprint.size() < 3) {return;}
  const auto world = transformFootprint(x, y, yaw, footprint);
  std::vector<cv::Point> polygon;
  for (const auto & p : world) {
    polygon.emplace_back(std::floor((p.x - map.getOriginX()) / map.getResolution()),
      std::floor((p.y - map.getOriginY()) / map.getResolution()));
  }
  cv::Mat grid(map.getSizeInCellsY(), map.getSizeInCellsX(), CV_8UC1, map.getCharMap());
  cv::fillPoly(grid, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(FREE_SPACE));
}
}  // namespace map_server
