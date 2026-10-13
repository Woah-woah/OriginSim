#include "map_server/dynamic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace map_server
{
DynamicLayer::DynamicLayer(rclcpp::Node & node) : node_(node)
{
  enabled_ = node.declare_parameter("dynamic_obstacles.enabled", false);
  timeout_ = node.declare_parameter("dynamic_obstacles.message_timeout", 0.5);
  observation_timeout_ = node.declare_parameter("dynamic_obstacles.observation_timeout", 0.6);
  if (!std::isfinite(timeout_) || !std::isfinite(observation_timeout_) || timeout_ <= 0 ||
    observation_timeout_ <= 0) {throw std::invalid_argument("Invalid dynamic layer timeout");}
  const auto topic = node.declare_parameter("dynamic_obstacles.topic", std::string("/dynamic_map/obstacles_raw"));
  if (!enabled_) {return;}
  subscription_ = node.create_subscription<msg::DynamicObstacles>(topic,
    rclcpp::QoS(1).reliable().transient_local(), [this](msg::DynamicObstacles::ConstSharedPtr message) {
      try {
        const auto & m = *message;
        const auto & q = m.info.origin.orientation;
        const auto & p = m.info.origin.position;
        const size_t count = static_cast<size_t>(m.info.width) * m.info.height;
        if (m.header.frame_id.empty() || !m.info.width || !m.info.height || count > 1000000 ||
          m.data.size() != count || m.confidence.size() != count ||
          !std::isfinite(m.info.resolution) || m.info.resolution < 0.01 || m.info.resolution > 1 ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::abs(p.z) > 1e-6 ||
          !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
          std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 || std::abs(q.w - 1) > 1e-6)
        {throw std::invalid_argument("Invalid dynamic obstacle geometry or payload size");}
        for (size_t i = 0; i < count; ++i) {
          if ((m.data[i] != FREE_SPACE && m.data[i] != LETHAL_OBSTACLE) ||
            !std::isfinite(m.confidence[i]) || m.confidence[i] < 0 || m.confidence[i] > 1)
          {throw std::invalid_argument("Invalid dynamic cost or confidence");}
        }
        const auto stamp = rclcpp::Time(m.header.stamp, node_.get_clock()->get_clock_type());
        const auto observation = rclcpp::Time(m.observation_stamp, node_.get_clock()->get_clock_type());
        if (stamp.nanoseconds() < 0 || observation.nanoseconds() < 0 || observation > stamp ||
          (m.current && observation.nanoseconds() == 0))
        {throw std::invalid_argument("Invalid dynamic timestamps");}
        message_ = message; received_ = std::chrono::steady_clock::now(); error_.clear();
      } catch (const std::exception & e) {
        message_.reset(); error_ = e.what();
        RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
          "Rejected dynamic obstacles: %s", e.what());
      }
    });
}
void DynamicLayer::apply(Costmap2D & grid, tf2_ros::Buffer & tf,
  const std::string & frame, double transform_tolerance)
{
  if (!enabled_) {return;}
  if (!message_) {throw std::runtime_error(error_);}
  const auto & m = *message_;
  const double age = (node_.now() - rclcpp::Time(m.header.stamp, node_.get_clock()->get_clock_type())).seconds();
  const double observation_age = (node_.now() - rclcpp::Time(m.observation_stamp,
    node_.get_clock()->get_clock_type())).seconds();
  if (!m.current || age < -0.1 || age > timeout_ || observation_age < -0.1 ||
    observation_age > observation_timeout_ ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() > timeout_)
  {throw std::runtime_error("Dynamic obstacles unavailable or stale: " + m.status);}
  double tx = 0, ty = 0, yaw = 0;
  if (frame != m.header.frame_id) {
    const auto transform = tf.lookupTransform(frame, m.header.frame_id, tf2::TimePointZero);
    const auto stamp = rclcpp::Time(transform.header.stamp, node_.get_clock()->get_clock_type());
    const double tf_age = (node_.now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (tf_age < -0.1 || tf_age > transform_tolerance))
    {throw std::runtime_error("Stale dynamic obstacle TF");}
    tx = transform.transform.translation.x; ty = transform.transform.translation.y;
    yaw = tf2::getYaw(transform.transform.rotation);
    if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(yaw))
    {throw std::runtime_error("Invalid dynamic obstacle TF");}
  }
  const double c = std::cos(yaw), s = std::sin(yaw), r = m.info.resolution;
  for (unsigned int y = 0; y < m.info.height; ++y) {
    for (unsigned int x = 0; x < m.info.width; ++x) {
      if (m.data[static_cast<size_t>(y) * m.info.width + x] != LETHAL_OBSTACLE) {continue;}
      const double wx = m.info.origin.position.x + x * r, wy = m.info.origin.position.y + y * r;
      double min_x = INFINITY, min_y = INFINITY, max_x = -INFINITY, max_y = -INFINITY;
      for (const auto & corner : {std::pair<double, double>{wx, wy}, {wx + r, wy},
        {wx, wy + r}, {wx + r, wy + r}})
      {
        const double px = tx + c * corner.first - s * corner.second;
        const double py = ty + s * corner.first + c * corner.second;
        min_x = std::min(min_x, px); max_x = std::max(max_x, px);
        min_y = std::min(min_y, py); max_y = std::max(max_y, py);
      }
      // Conservatively rasterize the transformed cell bounds, so a thin
      // obstacle cannot vanish when producer and consumer grids are offset.
      const double resolution = grid.getResolution();
      const double first_x = std::max(0.0, std::floor((min_x - grid.getOriginX()) / resolution + 1e-7));
      const double first_y = std::max(0.0, std::floor((min_y - grid.getOriginY()) / resolution + 1e-7));
      const double last_x = std::min(static_cast<double>(grid.getSizeInCellsX()) - 1,
        std::ceil((max_x - grid.getOriginX()) / resolution - 1e-7) - 1);
      const double last_y = std::min(static_cast<double>(grid.getSizeInCellsY()) - 1,
        std::ceil((max_y - grid.getOriginY()) / resolution - 1e-7) - 1);
      if (first_x > last_x || first_y > last_y) {continue;}
      for (unsigned int gy = first_y; gy <= last_y; ++gy) {
        for (unsigned int gx = first_x; gx <= last_x; ++gx) {grid.setCost(gx, gy, LETHAL_OBSTACLE);}
      }
    }
  }
}
}  // namespace map_server
