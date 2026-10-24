#pragma once
#include "map_server/map_server.hpp"
#include "map_server/msg/radar_targets.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
namespace map_server
{
RegisteredCloudFrame adaptCloud(const sensor_msgs::msg::PointCloud2 &input, tf2_ros::Buffer &tf,
                                const std::string &robot_frame, double received, size_t max_points);
RadarTargetFrame adaptRadar(const msg::RadarTargets &input, RadarSource source, double received);
} // namespace map_server
