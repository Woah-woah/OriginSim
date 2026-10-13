#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "map_server/dynamic_cloud_filter.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"
#include "map_server/cost_values.hpp"

namespace map_server
{
namespace
{
double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string pcdPath(const std::string & path)
{
  if (path.empty()) {throw std::invalid_argument("static_pcd must specify a PCD file");}
  const std::filesystem::path file(path);
  if (file.is_absolute() || file.has_parent_path()) {return path;}
  return ament_index_cpp::get_package_share_directory("point_lio_with_grid_map") + "/PCD/" + path;
}
Eigen::Isometry3d eigenTransform(const geometry_msgs::msg::TransformStamped & value)
{
  const auto & p = value.transform.translation;
  const auto & q = value.transform.rotation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 0.01)
  {throw std::runtime_error("Invalid point cloud TF");}
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return result;
}
void validateCloud(const sensor_msgs::msg::PointCloud2 & cloud, size_t maximum_points)
{
  const size_t count = static_cast<size_t>(cloud.width) * cloud.height;
  if (cloud.header.frame_id.empty() || cloud.is_bigendian || count > maximum_points ||
    static_cast<size_t>(cloud.row_step) * cloud.height != cloud.data.size() ||
    static_cast<size_t>(cloud.width) * cloud.point_step > cloud.row_step || cloud.point_step == 0)
  {throw std::invalid_argument("Invalid PointCloud2 geometry, size or endianness");}
  for (const std::string name : {"x", "y", "z"}) {
    bool found = false;
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
          field.offset > cloud.point_step || cloud.point_step - field.offset < sizeof(float))
        {throw std::invalid_argument("PointCloud2 XYZ fields must be valid FLOAT32 scalars");}
        found = true;
      }
    }
    if (!found) {throw std::invalid_argument("PointCloud2 is missing " + name);}
  }
}
}  // namespace

class DynamicMapNode : public rclcpp::Node
{
public:
  DynamicMapNode() : Node("dynamic_map")
  {
    enabled_ = declare_parameter("enabled", true);
    tracking_frame_ = declare_parameter("tracking_frame", std::string("odom"));
    static_frame_ = declare_parameter("static_pcd_frame", std::string("map"));
    robot_frame_ = declare_parameter("robot_base_frame", std::string("base_link_fake"));
    const auto cloud_topic = declare_parameter("cloud_topic", std::string("/cloud_registered"));
    const auto static_pcd = declare_parameter("static_pcd", std::string{});
    resolution_ = declare_parameter("resolution", 0.05);
    frequency_ = declare_parameter("update_frequency", 10.0);
    input_timeout_ = declare_parameter("input_timeout", 0.5);
    tf_wait_ = declare_parameter("tf_wait_timeout", 0.15);
    processing_timeout_ = declare_parameter("processing_timeout", 0.25);
    max_frames_ = declare_parameter("accumulate_frames", 4);
    accumulate_duration_ = declare_parameter("accumulate_duration", 0.35);
    max_points_ = declare_parameter("max_cloud_points", 200000);
    max_accumulated_ = declare_parameter("max_accumulated_points", 500000);
    DynamicFilterOptions filter;
    filter.roi_min_radius = declare_parameter("roi.min_radius", 0.4);
    filter.roi_max_radius = declare_parameter("roi.max_radius", 8.0);
    filter.roi_min_z = declare_parameter("roi.min_z", -0.45);
    filter.roi_max_z = declare_parameter("roi.max_z", 0.8);
    filter.voxel_size = declare_parameter("voxel_size", 0.03);
    filter.static_voxel_size = declare_parameter("static_voxel_size", 0.05);
    filter.static_distance = declare_parameter("static_distance_threshold", 0.2);
    filter.sor_neighbors = declare_parameter("sor.neighbors", 12);
    filter.sor_stddev = declare_parameter("sor.stddev_multiplier", 2.0);
    filter.min_density = declare_parameter("min_density_per_m2", 400.0);
    filter.min_component_area = declare_parameter("min_component_area", 0.0075);
    filter.component_connection_distance = declare_parameter("component_connection_distance", 0.55);
    DynamicMemoryOptions memory;
    memory.hit_rate = declare_parameter("memory.hit_rate", 5.0);
    memory.decay_rate = declare_parameter("memory.decay_rate", 0.7);
    memory.on_threshold = declare_parameter("memory.on_threshold", 0.6);
    memory.off_threshold = declare_parameter("memory.off_threshold", 0.25);
    memory.obstacle_timeout = declare_parameter("memory.obstacle_timeout", 1.0);
    memory.maximum_evidence_dt = declare_parameter("memory.maximum_evidence_dt", 0.2);
    for (double value : {resolution_, frequency_, input_timeout_, tf_wait_,
      processing_timeout_, accumulate_duration_})
    {if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid dynamic rate or timeout");}}
    if (resolution_ < 0.01 || resolution_ > 1 || frequency_ > 100 || input_timeout_ < tf_wait_ ||
      max_frames_ < 1 || max_frames_ > 20 || max_points_ < 1 || max_points_ > 2000000 ||
      max_accumulated_ < max_points_ || max_accumulated_ > 5000000 ||
      tracking_frame_.empty() || static_frame_.empty() || robot_frame_.empty() || cloud_topic.empty())
    {throw std::invalid_argument("Invalid dynamic bounds, frames or topics");}
    memory.nominal_dt = 1.0 / frequency_;
    memory_ = std::make_unique<DynamicMemory>(memory);
    const double cells = std::ceil(2 * filter.roi_max_radius / resolution_);
    if (!std::isfinite(cells) || cells < 1 || cells * cells > 1000000)
    {throw std::invalid_argument("Excessive dynamic grid dimensions");}
    grid_ = DynamicGrid{0, 0, static_cast<unsigned int>(cells), static_cast<unsigned int>(cells), resolution_};
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<msg::DynamicObstacles>("/dynamic_map/obstacles_raw", qos);
    visual_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dynamic_map/obstacles", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/dynamic_map/status", qos);
    const bool debug = declare_parameter("debug_clouds", false);
    if (debug) {
      accumulated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_map/accumulated_points", 1);
      filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_map/filtered_points", 1);
    }
    if (enabled_) {
      const auto load_begin = steadySeconds();
      filter_ = std::make_unique<DynamicCloudFilter>(filter, pcdPath(static_pcd));
      RCLCPP_INFO(get_logger(), "Static PCD KDTree: %zu points, %.3f s, frame=%s",
        filter_->staticSize(), steadySeconds() - load_begin, static_frame_.c_str());
      tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          try {
            validateCloud(*cloud, max_points_);
            const double stamp = rclcpp::Time(cloud->header.stamp).seconds();
            if (stamp <= 0 || stamp <= accepted_stamp_ ||
              (pending_ && stamp <= rclcpp::Time(pending_->header.stamp).seconds()) ||
              (latest_ && stamp <= rclcpp::Time(latest_->header.stamp).seconds()))
            {throw std::invalid_argument("PointCloud2 timestamp is zero, duplicate or out of order");}
            // Retain the frame waiting for TF, while keeping only the newest
            // incoming frame separately. High-rate inputs must not starve TF.
            latest_ = cloud; latest_received_ = steadySeconds();
          } catch (const std::exception & e) {
            pending_.reset(); latest_.reset(); frames_.clear(); fault_ = e.what();
          }
        });
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), [this]() {update();});
  }
private:
  struct Frame {double stamp; DynamicCloud::ConstPtr cloud;};
  Eigen::Isometry3d transform(const std::string & target, const std::string & source,
    const rclcpp::Time & stamp)
  {
    if (target == source) {return Eigen::Isometry3d::Identity();}
    return eigenTransform(tf_->lookupTransform(target, source, stamp, rclcpp::Duration::from_seconds(0)));
  }
  void center(const Eigen::Vector3d & robot)
  {
    const double x = std::floor(robot.x() / resolution_) - grid_.width / 2;
    const double y = std::floor(robot.y() / resolution_) - grid_.height / 2;
    if (!robot.allFinite() || std::abs(x) > 100000000 || std::abs(y) > 100000000)
    {throw std::runtime_error("Invalid dynamic robot position");}
    grid_.x = static_cast<int>(x); grid_.y = static_cast<int>(y);
  }
  void process(double time, double steady_time)
  {
    if (!pending_ && latest_) {
      pending_ = std::move(latest_); pending_received_ = latest_received_;
    }
    if (!pending_) {return;}
    const rclcpp::Time stamp(pending_->header.stamp, get_clock()->get_clock_type());
    const double observed = stamp.seconds();
    if (time - observed > input_timeout_ || observed > time + 0.1 ||
      steady_time - pending_received_ > input_timeout_)
    {fault_ = "Stale or future-dated input cloud"; pending_.reset(); frames_.clear(); return;}
    if (observed > time) {fault_ = "Waiting for cloud timestamp"; return;}
    try {
      const auto tracking_from_cloud = transform(tracking_frame_, pending_->header.frame_id, stamp);
      const auto tracking_from_robot = transform(tracking_frame_, robot_frame_, stamp);
      const auto static_from_tracking = transform(static_frame_, tracking_frame_, stamp);
      center(tracking_from_robot.translation());
      DynamicCloud input;
      pcl::fromROSMsg(*pending_, input);
      if (!input.empty() && std::none_of(input.begin(), input.end(), [](const pcl::PointXYZ & p) {
          return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }))
      {throw std::invalid_argument("Cloud contains no finite XYZ measurements");}
      auto fresh = filter_->prepare(input, tracking_from_cloud, tracking_from_robot.translation());
      while (!frames_.empty() && (observed - frames_.front().stamp > accumulate_duration_ ||
        frames_.size() >= static_cast<size_t>(max_frames_))) {frames_.pop_front();}
      frames_.push_back(Frame{observed, fresh});
      DynamicCloud accumulated;
      for (const auto & frame : frames_) {
        if (accumulated.size() + frame.cloud->size() > static_cast<size_t>(max_accumulated_))
        {throw std::runtime_error("Accumulated cloud exceeds point budget");}
        accumulated += *frame.cloud;
      }
      auto detection = filter_->detect(accumulated, *fresh, static_from_tracking,
        tracking_from_robot.translation(), grid_);
      const double finished = steadySeconds();
      if (finished - steady_time > processing_timeout_ ||
        finished - pending_received_ > input_timeout_ || now().seconds() - observed > input_timeout_)
      {throw std::runtime_error("Dynamic cloud processing deadline exceeded");}
      memory_->observe(detection.cells, observed, pending_received_);
      accepted_stamp_ = observed; received_ = pending_received_; fault_.clear();
      publishCloud(detection.accumulated, stamp, accumulated_pub_);
      publishCloud(detection.filtered, stamp, filtered_pub_);
      pending_.reset();
    } catch (const tf2::TransformException & e) {
      if (steadySeconds() - pending_received_ > tf_wait_) {
        fault_ = std::string("Cloud/robot/static PCD TF unavailable: ") + e.what();
        pending_.reset(); frames_.clear();
      } else if (accepted_stamp_ == 0 || !fault_.empty()) {
        fault_ = std::string("Waiting for cloud TF: ") + e.what();
      }
    } catch (const std::exception & e) {
      fault_ = e.what(); pending_.reset(); frames_.clear();
    }
  }
  void publishCloud(const DynamicCloud::ConstPtr & cloud, const rclcpp::Time & stamp,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    if (!publisher || publisher->get_subscription_count() == 0) {return;}
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*cloud, message); message.header.frame_id = tracking_frame_;
    message.header.stamp = stamp; publisher->publish(message);
  }
  void update()
  {
    const double time = now().seconds(), steady_time = steadySeconds();
    if (!memory_->advance(time, steady_time)) {
      frames_.clear(); pending_.reset(); latest_.reset(); accepted_stamp_ = 0; received_ = 0;
      fault_ = "ROS clock moved backwards; waiting for new cloud";
    }
    if (enabled_) {process(time, steady_time);}
    bool current = !enabled_ || (accepted_stamp_ > 0 && fault_.empty() &&
      time - accepted_stamp_ <= input_timeout_ && time - accepted_stamp_ >= -0.1 &&
      steadySeconds() - received_ <= input_timeout_);
    if (enabled_ && !current && fault_.empty()) {fault_ = "Point cloud input timed out";}
    msg::DynamicObstacles message;
    message.header.frame_id = tracking_frame_; message.header.stamp = now();
    message.observation_stamp = rclcpp::Time(static_cast<int64_t>((enabled_ ? accepted_stamp_ : time) * 1e9),
      get_clock()->get_clock_type());
    message.info.resolution = resolution_; message.info.width = grid_.width; message.info.height = grid_.height;
    message.info.origin.position.x = grid_.x * resolution_; message.info.origin.position.y = grid_.y * resolution_;
    message.info.origin.orientation.w = 1;
    auto snapshot = memory_->snapshot(grid_);
    message.data = std::move(snapshot.data); message.confidence = std::move(snapshot.confidence);
    message.current = current; message.status = !enabled_ ? "disabled" : current ? "ok" : fault_;
    publisher_->publish(message);
    nav_msgs::msg::OccupancyGrid visual;
    visual.header = message.header; visual.info = message.info;
    visual.data.reserve(message.data.size());
    for (const auto cost : message.data) {visual.data.push_back(cost == LETHAL_OBSTACLE ? 100 : 0);}
    visual_pub_->publish(visual);
    std_msgs::msg::String status; status.data = message.status; status_pub_->publish(status);
    if (!current) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message.status.c_str());}
  }
  bool enabled_;
  std::string tracking_frame_, static_frame_, robot_frame_, fault_{"Waiting for point cloud"};
  double resolution_, frequency_, input_timeout_, tf_wait_, processing_timeout_, accumulate_duration_;
  int max_frames_, max_points_, max_accumulated_;
  double accepted_stamp_{0}, received_{0}, pending_received_{0}, latest_received_{0};
  DynamicGrid grid_;
  std::deque<Frame> frames_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_, latest_;
  std::unique_ptr<DynamicMemory> memory_;
  std::unique_ptr<DynamicCloudFilter> filter_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<msg::DynamicObstacles>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pub_, filtered_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace map_server
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); int result = 0;
  try {rclcpp::spin(std::make_shared<map_server::DynamicMapNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamic_map"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
