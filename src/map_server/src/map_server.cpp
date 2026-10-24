#include "map_server/map_server.hpp"
#include "map_server/edt_2d.hpp"
#include "map_server/radar_rasterizer.hpp"
#include <algorithm>
#include <chrono>
namespace map_server
{
namespace
{
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
} // namespace
MapServer::MapServer(MapServerConfig c)
    : config_(c), memory_(c.cloud_memory), allies_(c.radar), enemies_(c.radar)
{
  c.cost.validate();
  for (double v : {c.processing_timeout, c.cloud_timeout, c.accumulate_duration, c.update_frequency,
                   c.localization_jump_m, c.localization_jump_rad})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid MapServer timing/jump option");
  if (c.update_frequency > 100 || !c.accumulate_frames || c.accumulate_frames > 100 ||
      !c.max_cloud_points || c.max_cloud_points > 10000000 ||
      c.max_accumulated_points < c.max_cloud_points || c.max_accumulated_points > 20000000)
    throw std::invalid_argument("Invalid cloud resource budget");
}
MapServer::~MapServer()
{
  stop();
}
bool MapServer::loadStaticMap(const StaticMapFiles &files)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_.joinable())
  {
    last_error_ = "Stop MapServer before loading maps";
    return false;
  }
  try
  {
    auto map = std::make_shared<StaticMap>(loadStaticMapFiles(files));
    std::unique_ptr<DynamicCloudFilter> filter;
    if (!files.static_pcd.empty())
      filter = std::make_unique<DynamicCloudFilter>(config_.cloud_filter, files.static_pcd);
    static_ = std::move(map);
    filter_ = std::move(filter);
    derived_.reset();
    resetLocked();
    last_error_.clear();
    return true;
  }
  catch (const std::exception &e)
  {
    last_error_ = e.what();
    return false;
  }
}
void MapServer::resetLocked()
{
  ++epoch_;
  ++input_version_;
  pending_cloud_.reset();
  worker_error_.clear();
  cloud_mask_.reset();
  derived_.reset();
  cloud_seen_ = false;
  cloud_error_ = false;
  cloud_stamp_ = 0;
  cloud_received_ = 0;
  accepted_cloud_ = -1;
  allies_.clear();
  enemies_.clear();
}
void MapServer::advanceTime(double now, double steady)
{
  if (!std::isfinite(now) || !std::isfinite(steady) || now < 0 || steady < 0)
    throw std::invalid_argument("Invalid source clocks");
  std::lock_guard<std::mutex> lock(mutex_);
  if (clock_seen_ && (now < ros_now_ || steady < steady_now_))
    resetLocked();
  ros_now_ = now;
  steady_now_ = steady;
  clock_seen_ = true;
  wake_.notify_one();
}
void MapServer::reportCloudError()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cloud_error_ = true;
  ++input_version_;
}
void MapServer::updateRegisteredCloud(const RegisteredCloudFrame &f)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (f.frame_id != "map" || !f.cloud || f.cloud->size() > config_.max_cloud_points ||
      !f.robot_map.allFinite() || !f.map_from_odom.matrix().allFinite() ||
      !std::isfinite(f.observation_stamp_s) || !std::isfinite(f.received_stamp_s) ||
      f.observation_stamp_s < 0 || f.received_stamp_s < 0 || !filter_ || !clock_seen_ ||
      f.observation_stamp_s > ros_now_ ||
      ros_now_ - f.observation_stamp_s > config_.cloud_timeout ||
      f.received_stamp_s > steady_now_ || steady_now_ - f.received_stamp_s > config_.cloud_timeout)
  {
    cloud_error_ = true;
    ++input_version_;
    return;
  }
  if (f.observation_stamp_s <= accepted_cloud_ ||
      (pending_cloud_ && f.observation_stamp_s <= pending_cloud_->observation_stamp_s))
    return;
  pending_cloud_ = f;
  accepted_cloud_ = f.observation_stamp_s;
  wake_.notify_one();
}
void MapServer::updateRadarAllies(const RadarTargetFrame &f)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clock_seen_ || f.observation_stamp_s > ros_now_ || f.received_stamp_s > steady_now_)
    throw std::invalid_argument("Radar clocks outside current clock domain");
  if (allies_.update(f, RadarSource::ALLY))
    ++input_version_;
  wake_.notify_one();
}
void MapServer::updateRadarEnemies(const RadarTargetFrame &f)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clock_seen_ || f.observation_stamp_s > ros_now_ || f.received_stamp_s > steady_now_)
    throw std::invalid_argument("Radar clocks outside current clock domain");
  if (enemies_.update(f, RadarSource::ENEMY))
    ++input_version_;
  wake_.notify_one();
}
DynamicHealth MapServer::healthLocked() const
{
  DynamicHealth h;
  h.map_error = !worker_error_.empty() ||
                (worker_steady_ > 0 && steady_now_ - worker_steady_ > config_.processing_timeout);
  h.cloud_observation_age_s = cloud_seen_ ? ros_now_ - cloud_stamp_ : INFINITY;
  h.cloud = cloud_error_   ? SourceStatus::ERROR
            : !cloud_seen_ ? SourceStatus::MISSING
            : (h.cloud_observation_age_s > config_.cloud_timeout ||
               steady_now_ - cloud_received_ > config_.cloud_timeout)
                ? SourceStatus::STALE
                : SourceStatus::VALID;
  h.radar_allies = allies_.status(ros_now_, steady_now_);
  h.radar_enemies = enemies_.status(ros_now_, steady_now_);
  h.ally_observation_age_s = allies_.age(ros_now_);
  h.enemy_observation_age_s = enemies_.age(ros_now_);
  h.self_filter_verified = allies_.selfFilterVerified();
  h.radar_time_degraded = allies_.timeDegraded() || enemies_.timeDegraded();
  return h;
}
std::string MapServer::getWorkerError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return worker_error_;
}
DynamicHealth MapServer::getDynamicHealth() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return healthLocked();
}
MapPublication MapServer::publication() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {derived_, healthLocked(), input_version_, worker_error_};
}
std::shared_ptr<const DynamicObstacleView> MapServer::getLatestDynamicObstacles() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto v = std::make_shared<DynamicObstacleView>();
  v->input_version = input_version_;
  v->cloud_mask = cloud_mask_;
  for (const auto &t : allies_.active(ros_now_, steady_now_))
    v->circles.push_back({t.xy_map, t.physical_radius_m});
  for (const auto &t : enemies_.active(ros_now_, steady_now_))
    v->circles.push_back({t.xy_map, t.physical_radius_m});
  return v;
}
std::shared_ptr<const StaticMap> MapServer::getStaticMap() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_;
}
std::shared_ptr<const DerivedMaps> MapServer::getDerivedMaps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return derived_;
}
RegionId MapServer::regionAt(const Eigen::Vector2d &p) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!static_)
    return INVALID_REGION;
  auto i = static_->geometry.index(p);
  return i ? static_->region_id.data[*i] : INVALID_REGION;
}
std::optional<float> MapServer::heightAt(const Eigen::Vector2d &p) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!static_ || !static_->height)
    return {};
  auto i = static_->geometry.index(p);
  return i ? std::optional<float>(static_->height->data[*i]) : std::nullopt;
}
std::optional<float> MapServer::depthAt(const Eigen::Vector2d &p) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!static_ || !static_->depth)
    return {};
  auto i = static_->geometry.index(p);
  return i ? std::optional<float>(static_->depth->data[*i]) : std::nullopt;
}
void MapServer::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_.joinable())
    return;
  if (!static_)
    throw std::logic_error("Load static map before start");
  stopping_ = false;
  worker_ = std::thread(&MapServer::run, this);
}
void MapServer::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    wake_.notify_all();
  }
  if (worker_.joinable())
    worker_.join();
}
void MapServer::run()
{
  uint64_t local_epoch = UINT64_MAX, version = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    version = derived_version_;
  }
  std::deque<RegisteredCloudFrame> frames;
  std::optional<Eigen::Isometry3d> previous_transform;
  DynamicGrid window{};
  std::vector<uint8_t> last_fused;
  while (true)
  {
    const auto cycle = Clock::now();
    std::optional<RegisteredCloudFrame> cloud;
    uint64_t epoch, input;
    double now, steady;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_)
        return;
      if (!clock_seen_)
      {
        wake_.wait(lock, [this] { return stopping_ || clock_seen_; });
        if (stopping_)
          return;
      }
      epoch = epoch_;
      input = input_version_;
      now = ros_now_;
      steady = steady_now_;
      cloud = std::move(pending_cloud_);
      pending_cloud_.reset();
    }
    if (epoch != local_epoch)
    {
      memory_.clear();
      frames.clear();
      previous_transform.reset();
      last_fused.clear();
      local_epoch = epoch;
      window = {};
    }
    double cloud_ms = 0;
    try
    {
      memory_.advance(now, steady);
      if (cloud)
      {
        const auto begun = Clock::now();
        const auto &f = *cloud;
        if (!f.cloud->empty() && std::none_of(f.cloud->begin(), f.cloud->end(),
                                              [](const auto &p) {
                                                return std::isfinite(p.x) && std::isfinite(p.y) &&
                                                       std::isfinite(p.z);
                                              }))
          throw std::invalid_argument("Cloud contains no finite measurements");
        if (now - f.observation_stamp_s > config_.cloud_timeout ||
            steady - f.received_stamp_s > config_.cloud_timeout)
          throw std::runtime_error("Cloud expired before processing");
        if (previous_transform)
        {
          auto delta = previous_transform->inverse() * f.map_from_odom;
          if (delta.translation().norm() > config_.localization_jump_m ||
              std::abs(Eigen::AngleAxisd(delta.linear()).angle()) > config_.localization_jump_rad)
          {
            memory_.clear();
            frames.clear();
          }
        }
        previous_transform = f.map_from_odom;
        const double r = static_->geometry.resolution;
        const double extent = std::ceil(2 * config_.cloud_filter.roi_max_radius / r) + 2;
        if (extent > 10000 || std::abs(f.robot_map.x() / r) > 1e8 ||
            std::abs(f.robot_map.y() / r) > 1e8)
          throw std::runtime_error("Excessive cloud grid extent");
        window = {static_cast<int>(std::floor(f.robot_map.x() / r) - extent / 2),
                  static_cast<int>(std::floor(f.robot_map.y() / r) - extent / 2),
                  static_cast<unsigned>(extent), static_cast<unsigned>(extent), r};
        auto fresh = filter_->prepare(*f.cloud, f.robot_map);
        auto saved = f;
        saved.cloud = fresh;
        while (!frames.empty() && (f.observation_stamp_s - frames.front().observation_stamp_s >
                                       config_.accumulate_duration ||
                                   frames.size() >= config_.accumulate_frames))
          frames.pop_front();
        frames.push_back(saved);
        DynamicCloud accumulated;
        for (const auto &frame : frames)
        {
          if (accumulated.size() + frame.cloud->size() > config_.max_accumulated_points)
            throw std::runtime_error("Cloud accumulation budget exceeded");
          accumulated += *frame.cloud;
        }
        auto detection = filter_->detect(accumulated, *fresh, f.robot_map, window);
        memory_.observe(detection.cells, f.observation_stamp_s, f.received_stamp_s);
        cloud_ms = elapsed(begun);
        std::lock_guard<std::mutex> lock(mutex_);
        if (epoch == epoch_)
        {
          cloud_seen_ = true;
          cloud_error_ = false;
          cloud_stamp_ = f.observation_stamp_s;
          cloud_received_ = f.received_stamp_s;
          ++input_version_;
        }
      }
      std::shared_ptr<const BinaryGrid> mask;
      if (window.width)
      {
        auto cells = memory_.snapshot(window);
        mask = std::make_shared<BinaryGrid>(
            BinaryGrid{{window.width, window.height, window.resolution,
                        window.x * window.resolution, window.y * window.resolution},
                       std::move(cells.data)});
      }
      std::shared_ptr<DynamicObstacleView> view = std::make_shared<DynamicObstacleView>();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (epoch != epoch_)
          continue;
        cloud_mask_ = mask;
        view->cloud_mask = mask;
        now = ros_now_;
        steady = steady_now_;
        worker_error_.clear();
        worker_steady_ = steady_now_;
        for (const auto &t : allies_.active(now, steady))
          view->circles.push_back({t.xy_map, t.physical_radius_m});
        for (const auto &t : enemies_.active(now, steady))
          view->circles.push_back({t.xy_map, t.physical_radius_m});
        view->input_version = input_version_;
        input = input_version_;
      }
      auto result = std::make_shared<DerivedMaps>();
      result->geometry = static_->geometry;
      result->dynamic_obstacles = view;
      const auto fusion_begin = Clock::now();
      result->fused_obstacles = static_->occupancy;
      if (mask)
        overlayConservative(result->fused_obstacles, *mask);
      rasterizeObstacles(result->fused_obstacles, view->circles);
      result->fusion_ms = elapsed(fusion_begin);
      const auto previous = getDerivedMaps();
      const bool obstacles_changed = result->fused_obstacles.data != last_fused;
      const bool dynamic_changed = !previous || !previous->dynamic_obstacles ||
                                   !sameDynamicGeometry(*previous->dynamic_obstacles, *view);
      if (obstacles_changed || dynamic_changed)
      {
        if (!obstacles_changed && previous)
        {
          // 地图外的雷达目标或窗口几何仍需发布，但沿用已经计算好的距离和浮点代价。
          result = std::make_shared<DerivedMaps>(*previous);
          result->dynamic_obstacles = view;
          result->edt_ms = 0;
          result->costmap_ms = 0;
        }
        else
        {
          const auto edt_begin = Clock::now();
          result->distance_field = preciseEdt(result->fused_obstacles);
          result->edt_ms = elapsed(edt_begin);
          const auto cost_begin = Clock::now();
          generateCosts(*result, config_.cost);
          result->costmap_ms = elapsed(cost_begin);
        }
        result->version = ++version;
        result->input_version = input;
        result->generated_at_s = now;
        result->cloud_ms = cloud_ms;
        std::lock_guard<std::mutex> lock(mutex_);
        if (epoch == epoch_)
        {
          last_fused = result->fused_obstacles.data;
          derived_version_ = result->version;
          derived_ = std::move(result);
        }
      }
    }
    catch (const std::exception &error)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (epoch == epoch_)
      {
        worker_error_ = error.what();
        if (cloud)
          cloud_error_ = true;
        frames.clear();
      }
    }
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_until(lock, cycle + std::chrono::duration<double>(1 / config_.update_frequency),
                     [this] { return stopping_; });
    if (stopping_)
      return;
  }
}
} // namespace map_server
