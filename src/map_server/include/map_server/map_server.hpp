#pragma once
#include "map_server/costmap_generator.hpp"
#include "map_server/dynamic_cloud_filter.hpp"
#include "map_server/static_map.hpp"
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
namespace map_server
{
struct RegisteredCloudFrame
{
  std::string frame_id{"map"};
  double observation_stamp_s{0}, received_stamp_s{0}; // ROS and steady clock, respectively
  DynamicCloud::ConstPtr cloud;
  Eigen::Vector3d robot_map{0, 0, 0};
  Eigen::Isometry3d map_from_odom{Eigen::Isometry3d::Identity()};
};
struct MapServerConfig
{
  CostmapOptions cost;
  RadarOptions radar;
  DynamicFilterOptions cloud_filter;
  DynamicMemoryOptions cloud_memory;
  double processing_timeout{0.25};
  double cloud_timeout{0.5}, accumulate_duration{0.35}, update_frequency{20};
  double localization_jump_m{0.5}, localization_jump_rad{0.35};
  size_t accumulate_frames{4}, max_cloud_points{200000}, max_accumulated_points{500000};
};
struct MapPublication
{
  std::shared_ptr<const DerivedMaps> maps;
  DynamicHealth health;
  uint64_t input_version{0};
  std::string worker_error;
};
class MapServer
{
public:
  explicit MapServer(MapServerConfig config = {});
  ~MapServer();
  bool loadStaticMap(const StaticMapFiles &files);
  const std::string &lastError() const
  {
    return last_error_;
  } // setup only, before start
  void updateRegisteredCloud(const RegisteredCloudFrame &frame_map);
  void updateRadarAllies(const RadarTargetFrame &frame_map);
  void updateRadarEnemies(const RadarTargetFrame &frame_map);
  RegionId regionAt(const Eigen::Vector2d &map_xy) const;
  std::optional<float> heightAt(const Eigen::Vector2d &map_xy) const;
  std::optional<float> depthAt(const Eigen::Vector2d &map_xy) const;
  std::shared_ptr<const StaticMap> getStaticMap() const;
  std::shared_ptr<const DerivedMaps> getDerivedMaps() const;
  std::shared_ptr<const DynamicObstacleView> getLatestDynamicObstacles() const;
  DynamicHealth getDynamicHealth() const;
  MapPublication publication() const;
  std::string getWorkerError() const;
  // ROS adapter supplies observation-clock now and monotonic reception-clock now.
  // A ROS-clock rollback resets every source atomically; it is not an observation.
  void advanceTime(double ros_now, double steady_now);
  void reportCloudError();
  void start();
  void stop();

private:
  DynamicHealth healthLocked() const;
  void resetLocked();
  void run();
  MapServerConfig config_;
  std::shared_ptr<const StaticMap> static_;
  std::unique_ptr<DynamicCloudFilter> filter_;
  DynamicMemory memory_;
  RadarTargetCache allies_, enemies_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  bool stopping_{false}, clock_seen_{false}, cloud_seen_{false}, cloud_error_{false};
  double ros_now_{0}, steady_now_{0}, cloud_stamp_{0}, cloud_received_{0}, accepted_cloud_{-1};
  double worker_steady_{0};
  uint64_t input_version_{0}, epoch_{0}, derived_version_{0};
  std::optional<RegisteredCloudFrame> pending_cloud_;
  std::shared_ptr<const BinaryGrid> cloud_mask_;
  std::shared_ptr<const DerivedMaps> derived_;
  std::string last_error_, worker_error_;
};
} // namespace map_server
