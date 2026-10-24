#include "ament_index_cpp/get_package_share_directory.hpp"
#include "map_server/map_geometry.hpp"
#include "map_server/map_server.hpp"
#include "map_server/msg/derived_map.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"
#include "map_server/msg/map_status.hpp"
#include "map_server/msg/static_map.hpp"
#include "map_server/ros_adapters.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <limits>
namespace map_server
{
namespace
{
double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
nav_msgs::msg::MapMetaData metadata(const GridGeometry &g)
{
  nav_msgs::msg::MapMetaData m;
  m.width = g.width;
  m.height = g.height;
  m.resolution = g.resolution;
  m.origin.position.x = g.origin_x;
  m.origin.position.y = g.origin_y;
  m.origin.orientation.w = 1;
  return m;
}
msg::DynamicHealth healthMessage(const DynamicHealth &h, const std_msgs::msg::Header &header)
{
  msg::DynamicHealth out;
  out.header = header;
  out.cloud = static_cast<uint8_t>(h.cloud);
  out.radar_allies = static_cast<uint8_t>(h.radar_allies);
  out.radar_enemies = static_cast<uint8_t>(h.radar_enemies);
  out.cloud_observation_age_s = h.cloud_observation_age_s;
  out.ally_observation_age_s = h.ally_observation_age_s;
  out.enemy_observation_age_s = h.enemy_observation_age_s;
  out.self_filter_verified = h.self_filter_verified;
  out.radar_time_degraded = h.radar_time_degraded;
  out.map_error = h.map_error;
  return out;
}
} // namespace
class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode() : Node("map_server"), tf_(get_clock()), listener_(tf_)
  {
    MapServerConfig c;
    const auto geometry = declareMapSettings(*this);
    footprint_ = geometry.footprint;
    footprint_frame_ = geometry.footprint_frame;
    c.cost.robot_radius =
        std::max(geometry.robot_radius, calculateMinAndMaxDistances(geometry.footprint).second);
    c.cost.safety_margin = declare_parameter("safety_margin", 0.0);
    c.cost.grid_margin_factor = declare_parameter("grid_margin_factor", c.cost.grid_margin_factor);
    c.cost.inflation_radius = geometry.inflation_radius;
    c.cost.cost_scaling_factor = geometry.cost_scaling_factor;
    c.update_frequency = declare_parameter("update_frequency", 20.0);
    c.processing_timeout = declare_parameter("processing_timeout", 0.25);
    c.cloud_timeout = declare_parameter("cloud_timeout", 0.5);
    c.accumulate_duration = declare_parameter("accumulate_duration", 0.35);
    const int frames = declare_parameter("accumulate_frames", 4),
              max_points = declare_parameter("max_cloud_points", 200000),
              max_accumulated = declare_parameter("max_accumulated_points", 500000);
    if (frames <= 0 || max_points <= 0 || max_accumulated <= 0)
      throw std::invalid_argument("Cloud budgets must be positive");
    c.accumulate_frames = frames;
    c.max_cloud_points = max_points;
    c.max_accumulated_points = max_accumulated;
    c.localization_jump_m = declare_parameter("localization_jump_m", 0.5);
    c.localization_jump_rad = declare_parameter("localization_jump_rad", 0.35);
    c.cloud_filter.roi_min_radius = declare_parameter("roi.min_radius", 0.4);
    c.cloud_filter.roi_max_radius = declare_parameter("roi.max_radius", 8.0);
    c.cloud_filter.roi_min_z = declare_parameter("roi.min_z", -0.45);
    c.cloud_filter.roi_max_z = declare_parameter("roi.max_z", 0.8);
    c.cloud_filter.voxel_size = declare_parameter("voxel_size", 0.03);
    c.cloud_filter.static_voxel_size = declare_parameter("static_voxel_size", 0.05);
    c.cloud_filter.static_distance = declare_parameter("static_distance_threshold", 0.2);
    c.cloud_filter.sor_neighbors = declare_parameter("sor.neighbors", 12);
    c.cloud_filter.sor_stddev = declare_parameter("sor.stddev_multiplier", 2.0);
    c.cloud_filter.min_density = declare_parameter("min_density_per_m2", 400.0);
    c.cloud_filter.min_component_area = declare_parameter("min_component_area", 0.0075);
    c.cloud_filter.component_connection_distance =
        declare_parameter("component_connection_distance", 0.55);
    c.cloud_memory.hit_rate = declare_parameter("memory.hit_rate", 5.0);
    c.cloud_memory.decay_rate = declare_parameter("memory.decay_rate", 0.7);
    c.cloud_memory.on_threshold = declare_parameter("memory.on_threshold", 0.6);
    c.cloud_memory.off_threshold = declare_parameter("memory.off_threshold", 0.25);
    c.cloud_memory.obstacle_timeout = declare_parameter("memory.obstacle_timeout", 1.0);
    c.cloud_memory.nominal_dt = declare_parameter("memory.nominal_dt", 0.05);
    c.cloud_memory.maximum_evidence_dt = declare_parameter("memory.maximum_evidence_dt", 0.2);
    c.radar.stale_after = declare_parameter("radar.stale_after", 2.5);
    c.radar.confidence_half_life = declare_parameter("radar.confidence_half_life", 2.0);
    c.radar.target_max_age = declare_parameter("radar.target_max_age", 4.0);
    c.radar.active_confidence_min = declare_parameter("radar.active_confidence_min", 0.25);
    c.radar.uncertainty_rate = declare_parameter("radar.uncertainty_rate", 0.1);
    c.radar.uncertainty_max = declare_parameter("radar.uncertainty_max", 0.4);
    c.radar.max_physical_radius = declare_parameter("radar.max_physical_radius", 2.0);
    int self = declare_parameter("radar.self_id", -1);
    if (self > 65535 || self < -1)
      throw std::invalid_argument("Invalid self ID");
    if (self >= 0)
      c.radar.self_id = static_cast<uint16_t>(self);
    robot_ = geometry.robot_base_frame;
    max_points_ = c.max_cloud_points;
    tf_wait_ = declare_parameter("tf_wait_timeout", 0.15);
    if (!std::isfinite(tf_wait_) || tf_wait_ <= 0)
      throw std::invalid_argument("Invalid TF wait timeout");
    const bool dynamic_enabled = declare_parameter("dynamic_obstacles_enabled", false);
    StaticMapFiles files{declare_parameter("yaml_filename", std::string{}),
                         declare_parameter("region_yaml", std::string{}),
                         declare_parameter("height_yaml", std::string{}),
                         declare_parameter("depth_yaml", std::string{}),
                         declare_parameter("static_pcd", std::string{})};
    if (!dynamic_enabled)
    {
      files.static_pcd.clear();
    }
    else if (files.static_pcd.rfind("package://", 0) == 0)
    {
      const auto split = files.static_pcd.find('/', 10);
      if (split == std::string::npos)
      {
        throw std::invalid_argument("PCD package URI 缺少文件路径");
      }
      files.static_pcd =
          ament_index_cpp::get_package_share_directory(files.static_pcd.substr(10, split - 10)) +
          files.static_pcd.substr(split);
    }
    else if (!files.static_pcd.empty())
    {
      const std::filesystem::path file(files.static_pcd);
      // 沿用原输入规则：纯文件名从定位包的 PCD 目录读取，带目录的路径保持原义。
      if (!file.is_absolute() && !file.has_parent_path())
      {
        files.static_pcd = ament_index_cpp::get_package_share_directory("point_lio_with_grid_map") +
                           "/PCD/" + files.static_pcd;
      }
    }
    if (dynamic_enabled && files.static_pcd.empty())
    {
      throw std::invalid_argument("启用动态障碍必须提供与 map 对齐的参考 PCD");
    }
    server_ = std::make_unique<MapServer>(c);
    if (!server_->loadStaticMap(files))
      throw std::runtime_error(server_->lastError());
    static_ = server_->getStaticMap();

    if (files.region_yaml.empty())
      RCLCPP_INFO(get_logger(),
                  "未提供 Region 语义栅格，区域栅格查询返回 INVALID_REGION；拓扑 YAML 不受影响");
    if (dynamic_enabled && self < 0)
      RCLCPP_WARN(get_logger(), "雷达自身编号未配置，尚不能确认已排除自身目标");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    derived_pub_ = create_publisher<msg::DerivedMap>("/map_server/derived", qos);
    static_pub_ = create_publisher<msg::StaticMap>("/map_server/static", qos);
    status_pub_ = create_publisher<msg::MapStatus>("/map_server/status", qos);
    dynamic_pub_ = create_publisher<msg::DynamicObstacles>("/map_server/dynamic", qos);
    debug_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map_server/costmap", qos);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    esdf_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map_server/esdf_grid", qos);
    esdf_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { publishEsdf(); });
    publishStatic();
    if (dynamic_enabled)
    {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          declare_parameter("cloud_topic", std::string("/cloud_registered")),
          rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
          {
            const auto stamp = rclcpp::Time(m->header.stamp).nanoseconds();
            if (stamp <= last_cloud_stamp_ ||
                (latest_ && stamp <= rclcpp::Time(latest_->header.stamp).nanoseconds()) ||
                (pending_ && stamp <= rclcpp::Time(pending_->header.stamp).nanoseconds()))
              return;
            latest_ = m;
            latest_received_ = steadySeconds();
          });
      const auto radar = [this](RadarSource source, msg::RadarTargets::ConstSharedPtr m)
      {
        try
        {
          double s = steadySeconds();
          server_->advanceTime(now().seconds(), s);
          auto f = adaptRadar(*m, source, s);
          if (source == RadarSource::ALLY)
            server_->updateRadarAllies(f);
          else
            server_->updateRadarEnemies(f);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "雷达目标未通过校验：%s",
                                e.what());
        }
      };
      allies_sub_ = create_subscription<msg::RadarTargets>(
          declare_parameter("radar.allies_topic", std::string("/radar/allies")), 10,
          [radar](msg::RadarTargets::ConstSharedPtr m) { radar(RadarSource::ALLY, m); });
      enemies_sub_ = create_subscription<msg::RadarTargets>(
          declare_parameter("radar.enemies_topic", std::string("/radar/enemies")), 10,
          [radar](msg::RadarTargets::ConstSharedPtr m) { radar(RadarSource::ENEMY, m); });
    }
    else
    {
      RCLCPP_INFO(get_logger(), "当前仅使用静态地图，未启用点云及雷达动态障碍订阅");
    }
    server_->advanceTime(now().seconds(), steadySeconds());
    server_->start();
    timer_ = create_wall_timer(std::chrono::duration<double>(1 / c.update_frequency),
                               [this] { tick(); });
  }

private:
  void tick()
  {
    const double time = now().seconds(), steady = steadySeconds();
    if (time < last_time_)
    {
      latest_.reset();
      pending_.reset();
      last_cloud_stamp_ = -1;
      ++producer_id_;
      published_ = 0;
      visualized_ = 0;
      debug_version_ = 0;
      dynamic_version_ = 0;
      last_dynamic_.reset();
      publishStatic();
    }
    last_time_ = time;
    server_->advanceTime(time, steady);
    if (!pending_ && latest_)
    {
      pending_ = std::move(latest_);
      pending_received_ = latest_received_;
    }
    if (pending_)
    {
      try
      {
        auto f = adaptCloud(*pending_, tf_, robot_, pending_received_, max_points_);
        server_->updateRegisteredCloud(f);
        last_cloud_stamp_ = rclcpp::Time(pending_->header.stamp).nanoseconds();
        pending_.reset();
      }
      catch (const tf2::TransformException &e)
      {
        if (steady - pending_received_ > tf_wait_)
        {
          server_->reportCloudError();
          pending_.reset();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云观测时刻的坐标变换不可用：%s",
                               e.what());
        }
      }
      catch (const std::exception &e)
      {
        server_->reportCloudError();
        pending_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云数据未通过校验：%s", e.what());
      }
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp = now();
    // 派生距离、代价和动态障碍只能来自一次融合，禁止混用更新中的障碍。
    const auto publication = server_->publication();
    const auto maps = publication.maps;
    if (!maps || !maps->dynamic_obstacles)
    {
      return;
    }
    const auto dynamic = maps->dynamic_obstacles;
    const auto &health = publication.health;
    const auto &error = publication.worker_error;
    if (!error.empty())
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "地图派生计算异常：%s",
                            error.c_str());
    if (!last_dynamic_ || !sameDynamicGeometry(*last_dynamic_, *dynamic))
    {
      msg::DynamicObstacles out;
      out.header = header;
      out.producer_id = producer_id_;
      out.version = ++dynamic_version_;
      out.input_version = dynamic->input_version;
      if (dynamic->cloud_mask)
      {
        out.cloud_info = metadata(dynamic->cloud_mask->geometry);
        out.cloud_mask = dynamic->cloud_mask->data;
      }
      for (const auto &circle : dynamic->circles)
      {
        msg::ObstacleCircle c;
        c.center.x = circle.center.x();
        c.center.y = circle.center.y();
        c.radius_m = circle.radius_m;
        out.circles.push_back(c);
      }
      dynamic_pub_->publish(std::move(out));
      last_dynamic_ = dynamic;
    }
    if (maps && maps->version != published_)
    {
      msg::DerivedMap out;
      out.header = header;
      out.producer_id = producer_id_;
      out.info = metadata(maps->geometry);
      out.version = maps->version;
      out.dynamic_version = dynamic_version_;
      out.input_version = maps->input_version;
      out.fused_obstacles = maps->fused_obstacles.data;
      out.distance_field = maps->distance_field.data;
      out.costmap = maps->costmap.data;
      out.collision_mask = maps->collision_mask.data;
      out.generated_at_s = maps->generated_at_s;
      out.hard_radius_m = maps->hard_radius_m;
      out.grid_margin_m = maps->grid_margin_m;
      out.robot_radius_m = maps->robot_radius_m;
      out.inflation_radius_m = maps->inflation_radius_m;
      out.cost_scaling_factor = maps->cost_scaling_factor;
      out.fusion_ms = maps->fusion_ms;
      out.edt_ms = maps->edt_ms;
      out.costmap_ms = maps->costmap_ms;
      out.cloud_ms = maps->cloud_ms;
      derived_pub_->publish(std::move(out));
      published_ = maps->version;
    }
    // 晚于静态派生地图启动的 RViz 也要收到全量快照，不依赖下一次障碍变化。
    if (debug_pub_->get_subscription_count() > 0 && maps->version != debug_version_)
    {
      nav_msgs::msg::OccupancyGrid debug;
      debug.header = header;
      debug.info = metadata(maps->geometry);
      for (float v : maps->costmap.data)
        debug.data.push_back(
            std::isfinite(v) ? static_cast<int8_t>(std::clamp((v - 1) * 20.0f, 0.0f, 99.0f)) : 100);
      debug_pub_->publish(std::move(debug));
      debug_version_ = maps->version;
    }
    msg::MapStatus status;
    status.header = header;
    status.producer_id = producer_id_;
    status.derived_version = published_;
    status.dynamic_version = dynamic_version_;
    status.input_version = publication.input_version;
    status.health = healthMessage(health, header);
    status.worker_error = error;
    status_pub_->publish(std::move(status));
  }
  void publishEsdf()
  {
    if (esdf_pub_->get_subscription_count() == 0)
      return;
    const auto maps = server_->getDerivedMaps();
    if (!maps || maps->version == visualized_)
      return;
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = now();
    grid.info = metadata(maps->geometry);
    grid.data.reserve(maps->geometry.size());
    // bit-nav /rog_map/field uses raw distance as Intensity, with automatic
    // finite min/max bounds. Keep that mapping while rendering a grid texture.
    float minimum = std::numeric_limits<float>::infinity();
    float maximum = -std::numeric_limits<float>::infinity();
    for (float distance : maps->distance_field.data)
    {
      if (std::isfinite(distance))
      {
        minimum = std::min(minimum, distance);
        maximum = std::max(maximum, distance);
      }
    }
    const double range = maximum > minimum ? double(maximum) - minimum : 1e20;
    for (float distance : maps->distance_field.data)
    {
      grid.data.push_back(
          std::isfinite(distance)
              ? static_cast<int8_t>(
                    std::lround(100.0 * std::clamp((double(distance) - minimum) / range, 0.0, 1.0)))
              : -1);
    }
    esdf_pub_->publish(std::move(grid));
    visualized_ = maps->version;
  }
  void publishStatic()
  {
    msg::StaticMap out;
    out.header.frame_id = "map";
    out.header.stamp = now();
    out.producer_id = producer_id_;
    out.info = metadata(static_->geometry);
    out.footprint = footprint_;
    out.footprint_frame = footprint_frame_;
    out.occupancy = static_->occupancy.data;
    out.region_id = static_->region_id.data;
    if (static_->height)
      out.height = static_->height->data;
    if (static_->depth)
      out.depth = static_->depth->data;
    static_pub_->publish(std::move(out));
    nav_msgs::msg::OccupancyGrid debug;
    debug.header.frame_id = "map";
    debug.header.stamp = now();
    debug.info = metadata(static_->geometry);
    for (auto v : static_->occupancy.data)
      debug.data.push_back(v ? 100 : 0);
    map_pub_->publish(std::move(debug));
  }

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<MapServer> server_;
  std::shared_ptr<const StaticMap> static_;
  std::string robot_, footprint_frame_;
  Footprint footprint_;
  size_t max_points_;
  double tf_wait_;
  double latest_received_{0}, pending_received_{0}, last_time_{0};
  int64_t last_cloud_stamp_{-1};
  uint64_t published_{0};
  uint64_t producer_id_{
      static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_, pending_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<msg::RadarTargets>::SharedPtr allies_sub_, enemies_sub_;
  rclcpp::Publisher<msg::DerivedMap>::SharedPtr derived_pub_;
  uint64_t dynamic_version_{0};
  std::shared_ptr<const DynamicObstacleView> last_dynamic_;
  rclcpp::Publisher<msg::StaticMap>::SharedPtr static_pub_;
  rclcpp::Publisher<msg::MapStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::DynamicObstacles>::SharedPtr dynamic_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_, map_pub_;
  uint64_t visualized_{0}, debug_version_{0};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_pub_;
  rclcpp::TimerBase::SharedPtr timer_, esdf_timer_;
};
} // namespace map_server
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    rclcpp::spin(std::make_shared<map_server::MapServerNode>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_server"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
