#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "map_server/map_geometry.hpp"
#include "map_server/map_view.hpp"
#include "map_server/msg/derived_map.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"
#include "map_server/msg/map_status.hpp"
#include "map_server/msg/static_map.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <mutex>
namespace map_server
{
class MapClient : public rclcpp::Node
{
public:
  explicit MapClient(const std::string &name,
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  std::shared_ptr<const MapView> view() const;
  bool isCurrent() const;
  bool isCurrent(const MapView &view) const;
  const MapSettings &settings() const
  {
    return settings_;
  }
  const std::string &getGlobalFrameID() const
  {
    return settings_.global_frame;
  }
  const std::string &getBaseFrameID() const
  {
    return settings_.robot_base_frame;
  }
  const Footprint &getRobotFootprint() const
  {
    return settings_.footprint;
  }
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const
  {
    return tf_;
  }
  Footprint footprintInBaseFrame(
      const builtin_interfaces::msg::Time &stamp = builtin_interfaces::msg::Time()) const;
  bool
  getRobotPose(geometry_msgs::msg::PoseStamped &pose,
               const builtin_interfaces::msg::Time &stamp = builtin_interfaces::msg::Time()) const;

private:
  void reject(uint8_t layer, const std::exception &error);
  void publishLocked();
  MapSettings settings_;
  MapPolicy policy_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  mutable std::mutex mutex_;
  std::shared_ptr<const MapView> state_{std::make_shared<const MapView>()};
  std::shared_ptr<MapView> assembly_{std::make_shared<MapView>()};
  rclcpp::Subscription<msg::StaticMap>::SharedPtr static_sub_;
  rclcpp::Subscription<msg::DerivedMap>::SharedPtr derived_sub_;
  rclcpp::Subscription<msg::DynamicObstacles>::SharedPtr dynamic_sub_;
  rclcpp::Subscription<msg::MapStatus>::SharedPtr status_sub_;
};
} // namespace map_server
