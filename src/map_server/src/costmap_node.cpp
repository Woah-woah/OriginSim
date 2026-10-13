#include "map_server/costmap_layers.hpp"
#include "map_server/dynamic_layer.hpp"
#include "map_server/map_geometry.hpp"
#include "map_server/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace map_server
{
class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode() : Node("costmap_server")
  {
    settings_ = declareMapSettings(*this);
    rolling_ = declare_parameter("rolling_window", false);
    resolution_ = declare_parameter("resolution", 0.05);
    width_ = declare_parameter("width", 9.0); height_ = declare_parameter("height", 9.0);
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.5);
    clearing_ = declare_parameter("footprint_clearing_enabled", true);
    threshold_ = declare_parameter("lethal_cost_threshold", 100);
    trinary_ = declare_parameter("trinary_costmap", true);
    inflation_.radius = settings_.inflation_radius;
    inflation_.cost_scaling_factor = settings_.cost_scaling_factor;
    inflation_.inscribed_radius = calculateMinAndMaxDistances(settings_.footprint).first;
    inflation_.inflate_unknown = declare_parameter("inflate_unknown", false);
    inflation_.inflate_around_unknown = declare_parameter("inflate_around_unknown", false);
    const double update_frequency = declare_parameter("update_frequency", 10.0);
    const double publish_frequency = declare_parameter("publish_frequency", 5.0);
    for (double value : {resolution_, width_, height_, transform_tolerance_,
      update_frequency, publish_frequency})
    {if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid costmap dimensions or rates");}}
    if (update_frequency > 1000 || publish_frequency > update_frequency ||
      width_ / resolution_ * height_ / resolution_ > 100000000 || threshold_ < 1 || threshold_ > 100)
    {throw std::invalid_argument("Invalid costmap rate, size or occupancy threshold");}
    publication_period_ = 1.0 / publish_frequency;
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    dynamic_layer_ = std::make_unique<DynamicLayer>(*this);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    raw_pub_ = create_publisher<msg::Costmap>("costmap_raw", qos);
    visual_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", qos);
    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/map")), qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        try {
          source_ = occupancyToCostmap(*map, settings_.track_unknown, threshold_, trinary_);
          source_frame_ = map->header.frame_id; load_time_ = map->info.map_load_time;
          source_valid_ = true;
        } catch (const std::exception & e) {
          source_valid_ = false; RCLCPP_ERROR(get_logger(), "Rejected /map: %s", e.what());
        }
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency), [this]() {update();});
  }

private:
  geometry_msgs::msg::TransformStamped transform(const std::string & target, const std::string & source)
  {
    auto value = tf_->lookupTransform(target, source, tf2::TimePointZero);
    const auto stamp = rclcpp::Time(value.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > transform_tolerance_))
    {throw std::runtime_error("Stale TF: " + source + " -> " + target);}
    return value;
  }

  void update()
  {
    bool current = false;
    try {
      if (!source_valid_) {throw std::runtime_error("Waiting for valid /map");}
      const auto robot = transform(settings_.global_frame, settings_.robot_base_frame);
      const double rx = robot.transform.translation.x, ry = robot.transform.translation.y;
      const double yaw = tf2::getYaw(robot.transform.rotation);
      if (rolling_) {
        const auto source_from_local = transform(source_frame_, settings_.global_frame);
        const auto & p = source_from_local.transform.translation;
        const double angle = tf2::getYaw(source_from_local.transform.rotation);
        const double c = std::cos(angle), s = std::sin(angle);
        const unsigned int nx = std::max(1.0, std::round(width_ / resolution_));
        const unsigned int ny = std::max(1.0, std::round(height_ / resolution_));
        const double ox = std::floor((rx - nx * resolution_ / 2) / resolution_) * resolution_;
        const double oy = std::floor((ry - ny * resolution_ / 2) / resolution_) * resolution_;
        // Include a halo so obstacles just beyond the window still inflate into it.
        const double halo_cells = std::ceil(inflation_.radius / resolution_);
        if (halo_cells > 10000 || (nx + 2 * halo_cells) * (ny + 2 * halo_cells) > 100000000)
        {throw std::invalid_argument("Excessive inflation halo");}
        const unsigned int halo = halo_cells;
        Costmap2D padded(nx + 2 * halo, ny + 2 * halo, resolution_,
          ox - halo * resolution_, oy - halo * resolution_,
          settings_.track_unknown ? NO_INFORMATION : FREE_SPACE);
        for (unsigned int y = 0; y < padded.getSizeInCellsY(); ++y) {
          for (unsigned int x = 0; x < padded.getSizeInCellsX(); ++x) {
            double wx, wy; padded.mapToWorld(x, y, wx, wy);
            unsigned int sx, sy;
            if (source_.worldToMap(p.x + c * wx - s * wy, p.y + s * wx + c * wy, sx, sy))
            {padded.setCost(x, y, source_.getCost(sx, sy));}
          }
        }
        current = mergeDynamic(padded);
        if (clearing_) {clearFootprint(padded, rx, ry, yaw, settings_.footprint);}
        inflate(padded, inflation_);
        output_.resizeMap(nx, ny, resolution_, ox, oy);
        for (unsigned int y = 0; y < ny; ++y) {
          for (unsigned int x = 0; x < nx; ++x) {output_.setCost(x, y, padded.getCost(x + halo, y + halo));}
        }
      } else {
        if (source_frame_ != settings_.global_frame)
        {throw std::invalid_argument("Global costmap frame must match /map frame");}
        output_ = source_;
        current = mergeDynamic(output_);
        if (clearing_) {clearFootprint(output_, rx, ry, yaw, settings_.footprint);}
        inflate(output_, inflation_);
      }
    } catch (const std::exception & e) {
      current = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Costmap unavailable: %s", e.what());
    }
    const auto now_steady = std::chrono::steady_clock::now();
    if (current != last_current_ ||
      std::chrono::duration<double>(now_steady - last_publish_).count() >= publication_period_)
    {
      if (output_.getSizeInCellsX()) {publish(current);}
      last_current_ = current; last_publish_ = now_steady;
    }
  }

  bool mergeDynamic(Costmap2D & grid)
  {
    try {
      dynamic_layer_->apply(grid, *tf_, settings_.global_frame, transform_tolerance_);
      return true;
    } catch (const std::exception & e) {
      // Keep the static view and its inflation available for diagnostics even
      // before the first valid dynamic observation. Planning stays invalid.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dynamic layer unavailable; costmap current=false: %s", e.what());
      return false;
    }
  }

  void publish(bool current)
  {
    msg::Costmap raw;
    raw.header.frame_id = settings_.global_frame; raw.header.stamp = now();
    raw.info.map_load_time = load_time_;
    raw.info.width = output_.getSizeInCellsX(); raw.info.height = output_.getSizeInCellsY();
    raw.info.resolution = output_.getResolution();
    raw.info.origin.position.x = output_.getOriginX(); raw.info.origin.position.y = output_.getOriginY();
    raw.info.origin.orientation.w = 1.0;
    raw.data.assign(output_.getCharMap(), output_.getCharMap() + static_cast<size_t>(raw.info.width) * raw.info.height);
    raw.current = current; raw.robot_base_frame = settings_.robot_base_frame;
    raw.footprint = settings_.footprint; raw.robot_radius = settings_.robot_radius;
    raw.track_unknown_space = settings_.track_unknown;
    raw.inflation_radius = settings_.inflation_radius; raw.cost_scaling_factor = settings_.cost_scaling_factor;
    raw_pub_->publish(raw);
    nav_msgs::msg::OccupancyGrid visual;
    visual.header = raw.header; visual.info = raw.info; visual.data.reserve(raw.data.size());
    // Nav2's visual translation table. This view is not the raw planning input.
    for (auto cost : raw.data) {
      visual.data.push_back(cost == NO_INFORMATION ? -1 : cost == LETHAL_OBSTACLE ? 100 :
        cost == INSCRIBED_INFLATED_OBSTACLE ? 99 : cost == FREE_SPACE ? 0 : 1 + (97 * (cost - 1)) / 251);
    }
    visual_pub_->publish(visual);
  }

  MapSettings settings_;
  std::unique_ptr<DynamicLayer> dynamic_layer_;
  InflationOptions inflation_;
  Costmap2D source_, output_;
  std::string source_frame_;
  builtin_interfaces::msg::Time load_time_;
  bool rolling_, clearing_, trinary_, source_valid_{false}, last_current_{false};
  int threshold_;
  double resolution_, width_, height_, transform_tolerance_, publication_period_;
  std::chrono::steady_clock::time_point last_publish_{};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<msg::Costmap>::SharedPtr raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visual_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace map_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); int result = 0;
  try {rclcpp::spin(std::make_shared<map_server::CostmapNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap_server"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
