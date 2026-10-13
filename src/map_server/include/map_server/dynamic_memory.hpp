#pragma once

#include <cstdint>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace map_server
{
struct DynamicCell
{
  int x, y;
  bool operator==(const DynamicCell & other) const {return x == other.x && y == other.y;}
};
struct DynamicCellHash
{
  size_t operator()(const DynamicCell & cell) const
  {
    return static_cast<size_t>((static_cast<uint64_t>(static_cast<uint32_t>(cell.x)) << 32) |
           static_cast<uint32_t>(cell.y));
  }
};
struct DynamicGrid
{
  int x, y;  // Origin in fixed world-lattice cells, including negative coordinates.
  unsigned int width, height;
  double resolution;
  size_t size() const {return static_cast<size_t>(width) * height;}
  bool contains(const DynamicCell & cell) const
  {return cell.x >= x && cell.y >= y && cell.x < x + static_cast<int>(width) &&
         cell.y < y + static_cast<int>(height);}
  size_t index(const DynamicCell & cell) const
  {return static_cast<size_t>(cell.y - y) * width + static_cast<size_t>(cell.x - x);}
};
struct DynamicMemoryOptions
{
  double hit_rate{5.0};
  double decay_rate{0.7};
  double on_threshold{0.6};
  double off_threshold{0.25};
  double obstacle_timeout{1.0};
  double nominal_dt{0.1};
  double maximum_evidence_dt{0.2};
};
struct DynamicSnapshot
{
  std::vector<uint8_t> data;
  std::vector<float> confidence;
};
class DynamicMemory
{
public:
  explicit DynamicMemory(const DynamicMemoryOptions & options);
  void clear();
  // Returns false on a ROS clock rollback and clears all memory.
  bool advance(double time, double steady_time);
  // Each independently timestamped frame contributes at most one hit per cell.
  void observe(const std::vector<DynamicCell> & cells, double time, double steady_time);
  DynamicSnapshot snapshot(const DynamicGrid & grid);
  size_t size() const {return cells_.size();}
private:
  struct Evidence
  {
    double confidence{0}, ros_seen{0}, steady_seen{0};
    bool occupied{false};
  };
  DynamicMemoryOptions options_;
  std::unordered_map<DynamicCell, Evidence, DynamicCellHash> cells_;
  double time_{0}, steady_time_{0}, observation_time_{0};
  bool initialized_{false}, observed_{false};
};
}  // namespace map_server
