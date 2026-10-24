#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
namespace map_server
{
struct GridGeometry
{
  uint32_t width{0}, height{0};
  double resolution{0}, origin_x{0}, origin_y{0};
  size_t size() const
  {
    return static_cast<size_t>(width) * height;
  }
  void validate() const
  {
    if (!width || !height || size() > 100000000 || !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(origin_x + width * resolution) ||
        !std::isfinite(origin_y + height * resolution))
      throw std::invalid_argument("Invalid grid geometry");
  }
  bool operator==(const GridGeometry &b) const
  {
    return width == b.width && height == b.height && std::abs(resolution - b.resolution) < 1e-9 &&
           std::abs(origin_x - b.origin_x) < 1e-9 && std::abs(origin_y - b.origin_y) < 1e-9;
  }
  std::optional<size_t> index(const Eigen::Vector2d &p) const
  {
    if (!p.allFinite() || p.x() < origin_x || p.y() < origin_y ||
        p.x() >= origin_x + width * resolution || p.y() >= origin_y + height * resolution)
      return {};
    return static_cast<size_t>(std::floor((p.y() - origin_y) / resolution)) * width +
           static_cast<size_t>(std::floor((p.x() - origin_x) / resolution));
  }
  Eigen::Vector2d center(size_t i) const
  {
    if (i >= size())
      throw std::out_of_range("Grid index");
    return {origin_x + (i % width + 0.5) * resolution, origin_y + (i / width + 0.5) * resolution};
  }
};
} // namespace map_server
