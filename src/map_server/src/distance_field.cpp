#include "map_server/distance_field.hpp"

#include <limits>
#include <opencv2/imgproc.hpp>

namespace map_server
{
bool DistanceField::matches(const Costmap2D & grid) const
{
  return width == grid.getSizeInCellsX() && height == grid.getSizeInCellsY() &&
    resolution == grid.getResolution() && origin_x == grid.getOriginX() &&
    origin_y == grid.getOriginY();
}

void DistanceField::validate() const
{
  const size_t count = static_cast<size_t>(width) * height;
  if (!width || !height || count > 100000000 || distances.size() != count ||
    obstacles.size() != count || !std::isfinite(resolution) || resolution <= 0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(max_distance) || max_distance <= 0 ||
    !std::isfinite(robot_radius) || robot_radius <= 0 ||
    !std::isfinite(inflation_radius) || inflation_radius < 0 ||
    !std::isfinite(cost_scaling_factor) || cost_scaling_factor < 0)
  {throw std::invalid_argument("Invalid EDT geometry or payload");}
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(distances[i]) || distances[i] < 0 || distances[i] > max_distance ||
      ((obstacles[i] == LETHAL_OBSTACLE || obstacles[i] == NO_INFORMATION) && distances[i] != 0))
    {throw std::invalid_argument("Invalid EDT value or obstacle distance");}
  }
}

bool DistanceField::sample(double x, double y, DistanceSample & value) const
{
  value = {};
  const double ex = origin_x + width * resolution, ey = origin_y + height * resolution;
  if (!std::isfinite(x) || !std::isfinite(y) || x < origin_x || y < origin_y || x >= ex || y >= ey ||
    !width || !height) {return false;}
  const unsigned int cell_x = (x - origin_x) / resolution, cell_y = (y - origin_y) / resolution;
  const auto occupancy = obstacles[static_cast<size_t>(cell_y) * width + cell_x];
  if (occupancy == NO_INFORMATION) {return false;}
  if (occupancy == LETHAL_OBSTACLE) {return true;}
  // 栅格中心双线性插值；边缘半格保持常值，梯度与同一插值严格一致。
  const double ux = (x - origin_x) / resolution - 0.5;
  const double uy = (y - origin_y) / resolution - 0.5;
  const double cx = std::clamp(ux, 0.0, static_cast<double>(width - 1));
  const double cy = std::clamp(uy, 0.0, static_cast<double>(height - 1));
  const unsigned int x0 = std::floor(cx), y0 = std::floor(cy);
  const unsigned int x1 = std::min(x0 + 1, width - 1), y1 = std::min(y0 + 1, height - 1);
  const double tx = cx - x0, ty = cy - y0;
  const auto at = [&](unsigned int ix, unsigned int iy) {return distances[static_cast<size_t>(iy) * width + ix];};
  const double a = at(x0, y0), b = at(x1, y0), c = at(x0, y1), d = at(x1, y1);
  value.distance = (1 - ty) * ((1 - tx) * a + tx * b) + ty * ((1 - tx) * c + tx * d);
  value.dx = ux >= 0 && ux < width - 1 ? ((1 - ty) * (b - a) + ty * (d - c)) / resolution : 0;
  value.dy = uy >= 0 && uy < height - 1 ? ((1 - tx) * (c - a) + tx * (d - b)) / resolution : 0;
  // 发布窗口外并非已知自由；边界距离提供保守软代价，硬约束另行检查车体。
  const double boundary[] = {x - origin_x, ex - x, y - origin_y, ey - y};
  const auto nearest = std::min_element(std::begin(boundary), std::end(boundary));
  if (*nearest < value.distance) {
    value.distance = *nearest;
    const auto side = nearest - boundary;
    value.dx = side == 0 ? 1 : side == 1 ? -1 : 0;
    value.dy = side == 2 ? 1 : side == 3 ? -1 : 0;
  }
  return std::isfinite(value.distance) && value.distance >= 0 &&
    std::isfinite(value.dx) && std::isfinite(value.dy);
}

DistanceField computeDistanceField(const Costmap2D & map, bool include_unknown, float maximum)
{
  DistanceField result;
  result.width = map.getSizeInCellsX(); result.height = map.getSizeInCellsY();
  result.resolution = map.getResolution();
  result.origin_x = map.getOriginX(); result.origin_y = map.getOriginY();
  if (!result.width || !result.height) {throw std::invalid_argument("Empty obstacle map");}
  result.max_distance = maximum > 0 ? maximum :
    std::hypot(result.width, result.height) * result.resolution;
  const size_t count = static_cast<size_t>(result.width) * result.height;
  result.obstacles.assign(map.getCharMap(), map.getCharMap() + count);
  cv::Mat seeds(result.height, result.width, CV_8UC1, cv::Scalar(255));
  bool occupied = false;
  for (size_t i = 0; i < count; ++i) {
    if (result.obstacles[i] == LETHAL_OBSTACLE ||
      (include_unknown && result.obstacles[i] == NO_INFORMATION))
    {seeds.data[i] = 0; occupied = true;}
  }
  result.distances.resize(count, result.max_distance);
  if (occupied) {
    cv::Mat cells;
    cv::distanceTransform(seeds, cells, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    const float * data = cells.ptr<float>();
    for (size_t i = 0; i < count; ++i) {
      if (!std::isfinite(data[i]) || data[i] < 0) {throw std::runtime_error("OpenCV EDT failed");}
      result.distances[i] = std::min(result.max_distance,
        static_cast<float>(data[i] * result.resolution));
    }
  }
  return result;
}

DistanceField cropDistanceField(const DistanceField & padded, unsigned int halo,
  unsigned int width, unsigned int height, float maximum)
{
  if (width + 2 * halo != padded.width || height + 2 * halo != padded.height ||
    !std::isfinite(maximum) || maximum <= 0) {throw std::invalid_argument("Invalid EDT crop");}
  DistanceField result;
  result.width = width; result.height = height; result.resolution = padded.resolution;
  result.origin_x = padded.origin_x + halo * padded.resolution;
  result.origin_y = padded.origin_y + halo * padded.resolution;
  result.max_distance = maximum;
  result.distances.resize(static_cast<size_t>(width) * height);
  result.obstacles.resize(result.distances.size());
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t source = static_cast<size_t>(y + halo) * padded.width + x + halo;
      const size_t target = static_cast<size_t>(y) * width + x;
      result.distances[target] = std::min(maximum, padded.distances[source]);
      result.obstacles[target] = padded.obstacles[source];
    }
  }
  return result;
}
}  // namespace map_server
