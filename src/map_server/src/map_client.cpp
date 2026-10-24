#include "map_server/map_client.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
namespace map_server
{
namespace
{
double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
GridGeometry geometry(const nav_msgs::msg::MapMetaData &m)
{
  const auto &p = m.origin.position;
  const auto &q = m.origin.orientation;
  if (!std::isfinite(p.z) || std::abs(p.z) > 1e-9 || !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w) || std::abs(q.x) > 1e-9 || std::abs(q.y) > 1e-9 ||
      std::abs(q.z) > 1e-9 || std::abs(q.w - 1) > 1e-9)
    throw std::invalid_argument("Map is not axis-aligned");
  GridGeometry g{m.width, m.height, m.resolution, p.x, p.y};
  g.validate();
  return g;
}
void header(const std_msgs::msg::Header &h, uint64_t producer)
{
  if (h.frame_id != "map" || !producer)
    throw std::invalid_argument("Map frame/producer mismatch");
}
DynamicHealth health(const msg::DynamicHealth &m)
{
  if (m.cloud > 3 || m.radar_allies > 3 || m.radar_enemies > 3)
    throw std::invalid_argument("Invalid source health");
  for (double age :
       {m.cloud_observation_age_s, m.ally_observation_age_s, m.enemy_observation_age_s})
    if (std::isnan(age) || age < 0)
      throw std::invalid_argument("Invalid observation age");
  return {static_cast<SourceStatus>(m.cloud),
          static_cast<SourceStatus>(m.radar_allies),
          static_cast<SourceStatus>(m.radar_enemies),
          m.cloud_observation_age_s,
          m.ally_observation_age_s,
          m.enemy_observation_age_s,
          m.self_filter_verified,
          m.radar_time_degraded,
          m.map_error};
}
} // namespace
MapClient::MapClient(const std::string &name, const rclcpp::NodeOptions &options)
    : Node(name, options)
{
  settings_ = declareMapSettings(*this);
  policy_.timeout_s = declare_parameter("map_timeout", 1.0);
  policy_.require_cloud = declare_parameter("require_cloud", true);
  policy_.require_radar = declare_parameter("require_radar", false);
  transform_tolerance_ = declare_parameter("transform_tolerance", 0.5);
  if (!std::isfinite(policy_.timeout_s) || policy_.timeout_s <= 0 ||
      !std::isfinite(transform_tolerance_) || transform_tolerance_ <= 0)
    throw std::invalid_argument("Invalid map client timeout");
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  static_sub_ = create_subscription<msg::StaticMap>(
      "/map_server/static", qos,
      [this](msg::StaticMap::UniquePtr m)
      {
        try
        {
          header(m->header, m->producer_id);
          if (m->footprint != settings_.footprint ||
              m->footprint_frame != settings_.footprint_frame)
            throw std::invalid_argument("地图生产端与控制端的真实车体轮廓或轮廓坐标系不一致");
          auto g = geometry(m->info);
          const size_t n = g.size();
          if (m->occupancy.size() != n || m->region_id.size() != n ||
              (!m->height.empty() && m->height.size() != n) ||
              (!m->depth.empty() && m->depth.size() != n))
            throw std::invalid_argument("Static layer size mismatch");
          for (auto v : m->occupancy)
            if (v > 1)
              throw std::invalid_argument("Nonbinary static occupancy");
          for (const auto *values : {&m->height, &m->depth})
            for (float v : *values)
              if (!std::isfinite(v))
                throw std::invalid_argument("Invalid static scalar layer");
          auto map = std::make_shared<StaticMap>();
          map->geometry = g;
          map->occupancy = {g, std::move(m->occupancy)};
          map->region_id = {g, std::move(m->region_id)};
          if (!m->height.empty())
            map->height = Grid<float>{g, std::move(m->height)};
          if (!m->depth.empty())
            map->depth = Grid<float>{g, std::move(m->depth)};
          std::lock_guard<std::mutex> lock(mutex_);
          auto next = std::make_shared<MapView>(*assembly_);
          if (next->static_ && next->static_producer_ == m->producer_id)
          {
            const auto &old = *next->static_;
            if (!(old.geometry == map->geometry) || old.occupancy.data != map->occupancy.data ||
                old.region_id.data != map->region_id.data ||
                old.height.has_value() != map->height.has_value() ||
                old.depth.has_value() != map->depth.has_value() ||
                (old.height && old.height->data != map->height->data) ||
                (old.depth && old.depth->data != map->depth->data))
              throw std::invalid_argument("Static map cannot change within a producer instance");
            next->error_mask_ &= ~1;
            assembly_ = next;
            publishLocked();
            return;
          }
          next->static_ = std::move(map);
          next->static_producer_ = m->producer_id;
          next->error_mask_ &= ~1;
          assembly_ = next;
          publishLocked();
        }
        catch (const std::exception &e)
        {
          reject(1, e);
        }
      });
  derived_sub_ = create_subscription<msg::DerivedMap>(
      "/map_server/derived", qos,
      [this](msg::DerivedMap::UniquePtr m)
      {
        try
        {
          header(m->header, m->producer_id);
          auto g = geometry(m->info);
          const auto n = g.size();
          if (m->fused_obstacles.size() != n || m->distance_field.size() != n ||
              m->costmap.size() != n || m->collision_mask.size() != n || !m->version ||
              !std::isfinite(m->generated_at_s) || !std::isfinite(m->grid_margin_m) ||
              m->grid_margin_m < 0 || !std::isfinite(m->hard_radius_m) ||
              m->hard_radius_m < settings_.robot_radius)
            throw std::invalid_argument("Derived map size/envelope mismatch");
          const double body = std::max(settings_.robot_radius,
                                       calculateMinAndMaxDistances(settings_.footprint).second);
          if (!std::isfinite(m->robot_radius_m) || std::abs(m->robot_radius_m - body) > 1e-9 ||
              !std::isfinite(m->inflation_radius_m) ||
              std::abs(m->inflation_radius_m - settings_.inflation_radius) > 1e-9 ||
              !std::isfinite(m->cost_scaling_factor) ||
              std::abs(m->cost_scaling_factor - settings_.cost_scaling_factor) > 1e-9)
            throw std::invalid_argument("地图生产端与消费者的车体或距离代价参数不一致");
          for (size_t i = 0; i < n; ++i)
          {
            const float c = m->costmap[i], d = m->distance_field[i];
            if (m->fused_obstacles[i] > 1 || m->collision_mask[i] > 1 || !std::isfinite(d) ||
                d < 0 || std::isnan(c) || c < 1 ||
                m->collision_mask[i] != (std::isinf(c) ? 1 : 0) ||
                (m->fused_obstacles[i] && (d != 0 || !m->collision_mask[i])))
              throw std::invalid_argument("Invalid derived cell=" + std::to_string(i) + " cost=" +
                                          std::to_string(c) + " distance=" + std::to_string(d) +
                                          " obstacle=" + std::to_string(m->fused_obstacles[i]) +
                                          " collision=" + std::to_string(m->collision_mask[i]));
          }
          auto map = std::make_shared<DerivedMaps>();
          map->geometry = g;
          map->version = m->version;
          map->dynamic_version = m->dynamic_version;
          map->input_version = m->input_version;
          map->fused_obstacles = {g, std::move(m->fused_obstacles)};
          map->distance_field = {g, std::move(m->distance_field)};
          map->costmap = {g, std::move(m->costmap)};
          map->collision_mask = {g, std::move(m->collision_mask)};
          map->generated_at_s = m->generated_at_s;
          map->hard_radius_m = m->hard_radius_m;
          map->grid_margin_m = m->grid_margin_m;
          map->robot_radius_m = m->robot_radius_m;
          map->inflation_radius_m = m->inflation_radius_m;
          map->cost_scaling_factor = m->cost_scaling_factor;
          map->fusion_ms = m->fusion_ms;
          map->edt_ms = m->edt_ms;
          map->costmap_ms = m->costmap_ms;
          map->cloud_ms = m->cloud_ms;
          std::lock_guard<std::mutex> lock(mutex_);
          auto next = std::make_shared<MapView>(*assembly_);
          if (next->derived_ && next->derived_producer_ == m->producer_id &&
              next->derived_->version >= m->version && !(next->error_mask_ & 2))
            return;
          next->derived_ = std::move(map);
          next->derived_producer_ = m->producer_id;
          next->error_mask_ &= ~2;
          assembly_ = next;
          publishLocked();
        }
        catch (const std::exception &e)
        {
          reject(2, e);
        }
      });
  dynamic_sub_ = create_subscription<msg::DynamicObstacles>(
      "/map_server/dynamic", qos,
      [this](msg::DynamicObstacles::UniquePtr m)
      {
        try
        {
          header(m->header, m->producer_id);
          if (!m->version || m->circles.size() > 2000)
            throw std::invalid_argument("Invalid dynamic version/size");
          auto dynamic = std::make_shared<DynamicObstacleView>();
          dynamic->input_version = m->input_version;
          if (!m->cloud_mask.empty())
          {
            auto g = geometry(m->cloud_info);
            if (g.size() != m->cloud_mask.size())
              throw std::invalid_argument("Invalid cloud mask size");
            for (auto c : m->cloud_mask)
              if (c > 1)
                throw std::invalid_argument("Nonbinary cloud mask");
            dynamic->cloud_mask =
                std::make_shared<BinaryGrid>(BinaryGrid{g, std::move(m->cloud_mask)});
          }
          for (const auto &c : m->circles)
          {
            if (!std::isfinite(c.center.x) || !std::isfinite(c.center.y) ||
                !std::isfinite(c.radius_m) || c.radius_m < 0)
              throw std::invalid_argument("Invalid obstacle circle");
            dynamic->circles.push_back({{c.center.x, c.center.y}, c.radius_m});
          }
          const auto stamp = rclcpp::Time(m->header.stamp).seconds();
          std::lock_guard<std::mutex> lock(mutex_);
          auto next = std::make_shared<MapView>(*assembly_);
          if (next->dynamic_ && next->dynamic_producer_ == m->producer_id &&
              m->version <= next->dynamic_version_ && !(next->error_mask_ & 4))
            return;
          next->dynamic_ = std::move(dynamic);
          next->dynamic_producer_ = m->producer_id;
          next->dynamic_version_ = m->version;
          next->dynamic_stamp_s_ = stamp;
          next->error_mask_ &= ~4;
          assembly_ = next;
          publishLocked();
        }
        catch (const std::exception &e)
        {
          reject(4, e);
        }
      });
  status_sub_ = create_subscription<msg::MapStatus>(
      "/map_server/status", qos,
      [this](msg::MapStatus::UniquePtr m)
      {
        try
        {
          header(m->header, m->producer_id);
          MapStatusData status{health(m->health),  m->producer_id,
                               m->derived_version, m->dynamic_version,
                               m->input_version,   rclcpp::Time(m->header.stamp).seconds(),
                               steadySeconds()};
          if (!m->worker_error.empty())
          {
            status.health.map_error = true;
          }
          if (now().seconds() < status.stamp_s - 0.1 ||
              now().seconds() - status.stamp_s > policy_.timeout_s)
            throw std::invalid_argument("Stale map heartbeat");
          std::lock_guard<std::mutex> lock(mutex_);
          auto next = std::make_shared<MapView>(*assembly_);
          if (next->status_.producer_id == status.producer_id &&
              status.stamp_s < next->status_.stamp_s)
            return;
          next->status_ = status;
          next->error_mask_ &= ~8;
          assembly_ = next;
          publishLocked();
        }
        catch (const std::exception &e)
        {
          reject(8, e);
        }
      });
}
// Assemble independently delivered buffers using the producer's published
// watermarks. Only shared pointers are staged. Readers retain immutable,
// acknowledged buffers.
void MapClient::publishLocked()
{
  const auto &p = *assembly_;
  const bool same =
      p.static_ && p.derived_ && p.dynamic_ && p.static_producer_ == p.derived_producer_ &&
      p.static_producer_ == p.dynamic_producer_ && p.static_producer_ == p.status_.producer_id;
  if (!same || p.error_mask_)
  {
    state_ = std::make_shared<MapView>(p);
    return;
  }
  const bool failed =
      p.status_.health.map_error ||
      (policy_.require_cloud && p.status_.health.cloud != SourceStatus::VALID) ||
      (policy_.require_radar && (p.status_.health.radar_allies != SourceStatus::VALID ||
                                 p.status_.health.radar_enemies != SourceStatus::VALID ||
                                 !p.status_.health.self_filter_verified));
  if (!failed &&
      (p.status_.derived_version < p.version() || p.status_.dynamic_version < p.dynamic_version_))
    return;
  auto next = std::make_shared<MapView>(p);
  const double steady = steadySeconds();
  const bool previous = state_->status_.producer_id == p.status_.producer_id;
  const auto waiting =
      [&](uint64_t advertised, uint64_t received, uint64_t old_received, double since)
  {
    if (advertised <= received)
      return 0.0;
    return !previous || received > old_received || since == 0 ? steady : since;
  };
  next->derived_wait_since_ = waiting(p.status_.derived_version, p.version(), state_->version(),
                                      state_->derived_wait_since_);
  next->dynamic_wait_since_ = waiting(p.status_.dynamic_version, p.dynamic_version_,
                                      state_->dynamic_version_, state_->dynamic_wait_since_);
  state_ = std::move(next);
}
void MapClient::reject(uint8_t layer, const std::exception &error)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    assembly_->error_mask_ |= layer;
    auto next = std::make_shared<MapView>(*state_);
    next->error_mask_ |= layer;
    state_ = next;
  }
  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected map layer %u: %s", layer,
                        error.what());
}
std::shared_ptr<const MapView> MapClient::view() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}
bool MapClient::isCurrent() const
{
  return isCurrent(*view());
}
bool MapClient::isCurrent(const MapView &captured) const
{
  const auto latest = view();
  const double ros = now().seconds(), steady = steadySeconds();
  // 心跳可以刷新同一几何版本；生产者重启、地图更新或健康失效使旧快照立即不可用。
  return latest->current(ros, steady, policy_) && captured.current(ros, steady, policy_) &&
         captured.status().producer_id == latest->status().producer_id &&
         captured.version() == latest->version();
}
Footprint MapClient::footprintInBaseFrame(const builtin_interfaces::msg::Time &stamp) const
{
  if (settings_.footprint_frame == settings_.robot_base_frame)
  {
    return settings_.footprint;
  }
  const auto t = tf_->lookupTransform(settings_.robot_base_frame, settings_.footprint_frame,
                                      rclcpp::Time(stamp, get_clock()->get_clock_type()));
  const auto &q = t.transform.rotation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3 ||
      !std::isfinite(t.transform.translation.x) || !std::isfinite(t.transform.translation.y))
    throw std::runtime_error("真实车体轮廓坐标变换无效");
  // 冻结一次观测的真实车体相对朝向；不改变原有速度控制坐标系。
  return transformFootprint(t.transform.translation.x, t.transform.translation.y, tf2::getYaw(q),
                            settings_.footprint);
}
bool MapClient::getRobotPose(geometry_msgs::msg::PoseStamped &pose,
                             const builtin_interfaces::msg::Time &observation) const
{
  try
  {
    const auto t = tf_->lookupTransform("map", settings_.robot_base_frame,
                                        rclcpp::Time(observation, get_clock()->get_clock_type()));
    double stamp = rclcpp::Time(t.header.stamp, get_clock()->get_clock_type()).seconds();
    const double age = now().seconds() - stamp;
    if (stamp != 0 && (age < -0.1 || age > transform_tolerance_))
      return false;
    pose.header = t.header;
    pose.pose.position.x = t.transform.translation.x;
    pose.pose.position.y = t.transform.translation.y;
    pose.pose.position.z = t.transform.translation.z;
    pose.pose.orientation = t.transform.rotation;
    return true;
  }
  catch (const tf2::TransformException &)
  {
    return false;
  }
}
} // namespace map_server
