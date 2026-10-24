#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <vector>
#include "map_server/cost_values.hpp"

namespace map_server
{
struct DistanceField;

// Axis-aligned, row-major raw cost grid. Callers lock getMutex() for compound reads.
class Costmap2D
{
public:
  using mutex_t = std::recursive_mutex;
  Costmap2D() = default;
  Costmap2D(unsigned int width, unsigned int height, double resolution,
    double origin_x, double origin_y, unsigned char value = FREE_SPACE)
  {resizeMap(width, height, resolution, origin_x, origin_y, value);}
  Costmap2D(const Costmap2D & other) {*this = other;}
  Costmap2D & operator=(const Costmap2D & other)
  {
    if (this == &other) {return *this;}
    std::scoped_lock lock(mutex_, other.mutex_);
    width_ = other.width_; height_ = other.height_; resolution_ = other.resolution_;
    origin_x_ = other.origin_x_; origin_y_ = other.origin_y_; data_ = other.data_;
    distance_field_ = other.distance_field_;
    return *this;
  }
  void resizeMap(unsigned int width, unsigned int height, double resolution,
    double origin_x, double origin_y, unsigned char value = FREE_SPACE)
  {
    if (!width || !height || !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
      static_cast<size_t>(width) * height > 100000000)
    {throw std::invalid_argument("Invalid or excessive costmap geometry");}
    std::vector<unsigned char> data(static_cast<size_t>(width) * height, value);
    std::lock_guard<mutex_t> lock(mutex_);
    width_ = width; height_ = height; resolution_ = resolution;
    origin_x_ = origin_x; origin_y_ = origin_y; data_.swap(data);
    distance_field_.reset();
  }
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < origin_x_ || wy < origin_y_ ||
      wx >= origin_x_ + width_ * resolution_ || wy >= origin_y_ + height_ * resolution_)
    {return false;}
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    return mx < width_ && my < height_;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {wx = origin_x_ + (mx + 0.5) * resolution_; wy = origin_y_ + (my + 0.5) * resolution_;}
  unsigned char getCost(unsigned int mx, unsigned int my) const
  {return data_.at(static_cast<size_t>(my) * width_ + mx);}
  unsigned char getCost(unsigned int index) const {return data_.at(index);}
  void setCost(unsigned int mx, unsigned int my, unsigned char value)
  {data_.at(static_cast<size_t>(my) * width_ + mx) = value;}
  unsigned int getSizeInCellsX() const {return width_;}
  unsigned int getSizeInCellsY() const {return height_;}
  double getResolution() const {return resolution_;}
  double getOriginX() const {return origin_x_;}
  double getOriginY() const {return origin_y_;}
  unsigned char * getCharMap() {return data_.data();}
  const unsigned char * getCharMap() const {return data_.data();}
  // 与 uint8 地图在同一个互斥锁下复制，EDT 在消费者之间共享只读。
  std::shared_ptr<const DistanceField> distanceField() const {return distance_field_;}
  void setDistanceField(std::shared_ptr<const DistanceField> field) {distance_field_ = std::move(field);}
  mutex_t * getMutex() const {return &mutex_;}

private:
  unsigned int width_{0}, height_{0};
  double resolution_{1.0}, origin_x_{0.0}, origin_y_{0.0};
  std::vector<unsigned char> data_;
  std::shared_ptr<const DistanceField> distance_field_;
  mutable mutex_t mutex_;
};
}  // namespace map_server
