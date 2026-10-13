#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include "map_server/costmap_2d.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"

namespace map_server
{
// Validates the producer heartbeat and observation age before modifying any grid.
class DynamicLayer
{
public:
  explicit DynamicLayer(rclcpp::Node & node);
  void apply(Costmap2D & grid, tf2_ros::Buffer & tf, const std::string & frame,
    double transform_tolerance);
private:
  rclcpp::Node & node_;
  bool enabled_;
  double timeout_, observation_timeout_;
  msg::DynamicObstacles::ConstSharedPtr message_;
  std::chrono::steady_clock::time_point received_{};
  std::string error_{"Waiting for dynamic obstacles"};
  rclcpp::Subscription<msg::DynamicObstacles>::SharedPtr subscription_;
};
}  // namespace map_server
