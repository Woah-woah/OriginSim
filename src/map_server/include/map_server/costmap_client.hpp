#pragma once
#include <chrono>
#include "map_server/costmap_2d.hpp"
#include "map_server/map_geometry.hpp"
#include "map_server/msg/costmap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace map_server
{
static_assert(msg::Costmap::FREE_SPACE == FREE_SPACE);
static_assert(msg::Costmap::INSCRIBED_INFLATED_OBSTACLE == INSCRIBED_INFLATED_OBSTACLE);
static_assert(msg::Costmap::LETHAL_OBSTACLE == LETHAL_OBSTACLE);
static_assert(msg::Costmap::NO_INFORMATION == NO_INFORMATION);
// Consumer only: stores full snapshots received from the independent costmap process.
class CostmapClient : public rclcpp::Node
{
public:
  explicit CostmapClient(const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  Costmap2D * getCostmap() {return &grid_;}
  const MapSettings & settings() const {return settings_;}
  const std::string & getGlobalFrameID() const {return settings_.global_frame;}
  const std::string & getBaseFrameID() const {return settings_.robot_base_frame;}
  const Footprint & getRobotFootprint() const {return settings_.footprint;}
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf_;}
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);
  bool isCurrent() const;
  rclcpp::Time snapshotStamp() const;

private:
  MapSettings settings_;
  Costmap2D grid_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<msg::Costmap>::SharedPtr subscription_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool current_{false};
  double timeout_, transform_tolerance_;
};
}  // namespace map_server
