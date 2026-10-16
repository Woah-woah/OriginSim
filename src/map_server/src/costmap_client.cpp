#include "map_server/costmap_client.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace map_server
{
CostmapClient::CostmapClient(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options)
{
  settings_ = declareMapSettings(*this);
  timeout_ = declare_parameter("costmap_timeout", 1.0);
  transform_tolerance_ = declare_parameter("transform_tolerance", 0.5);
  if (!std::isfinite(timeout_) || timeout_ <= 0 ||
    !std::isfinite(transform_tolerance_) || transform_tolerance_ <= 0)
  {throw std::invalid_argument("Costmap and transform timeouts must be positive");}
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  subscription_ = create_subscription<msg::Costmap>(
    declare_parameter("costmap_topic", std::string("/global_costmap/costmap_raw")),
    rclcpp::QoS(1).reliable().transient_local(), [this](msg::Costmap::ConstSharedPtr message) {
      try {
        const auto & m = *message;
        const auto & p = m.info.origin.position;
        const auto & q = m.info.origin.orientation;
        if (m.header.frame_id != settings_.global_frame ||
          m.robot_base_frame != settings_.robot_base_frame ||
          !std::isfinite(p.z) || std::abs(p.z) > 1e-6 ||
          !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
          std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 ||
          std::abs(q.w - 1) > 1e-6 ||
          !m.info.width || !m.info.height ||
          m.data.size() != static_cast<size_t>(m.info.width) * m.info.height ||
          m.track_unknown_space != settings_.track_unknown ||
          m.robot_radius != settings_.robot_radius || m.inflation_radius != settings_.inflation_radius ||
          m.cost_scaling_factor != settings_.cost_scaling_factor || m.footprint != settings_.footprint)
        {throw std::invalid_argument("Costmap message geometry or settings mismatch");}
        Costmap2D next(m.info.width, m.info.height, m.info.resolution, p.x, p.y);
        std::copy(m.data.begin(), m.data.end(), next.getCharMap());
        const auto stamp = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
        if (stamp.nanoseconds() < 0) {throw std::invalid_argument("Negative map timestamp");}
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
        grid_ = next; stamp_ = stamp;
        received_ = std::chrono::steady_clock::now(); current_ = m.current;
      } catch (const std::exception & e) {
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex()); current_ = false;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected costmap: %s", e.what());
      }
    });
}

rclcpp::Time CostmapClient::snapshotStamp() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  return stamp_;
}

bool CostmapClient::isCurrent() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  const double age = (get_clock()->now() - stamp_).seconds();
  return current_ && age >= -0.1 && age <= timeout_ &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <= timeout_;
}

bool CostmapClient::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    const auto transform = tf_->lookupTransform(
      settings_.global_frame, settings_.robot_base_frame, tf2::TimePointZero);
    const auto stamp = rclcpp::Time(transform.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > transform_tolerance_)) {return false;}
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException &) {return false;}
}
}  // namespace map_server
