#include "map_server/ros_adapters.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
namespace map_server
{
namespace
{
Eigen::Isometry3d eigen(const geometry_msgs::msg::TransformStamped &t)
{
  const auto &q = t.transform.rotation;
  const auto &p = t.transform.translation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1) > 1e-3 ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    throw std::invalid_argument("Invalid cloud TF");
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = rotation.normalized().toRotationMatrix();
  out.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return out;
}
} // namespace
RegisteredCloudFrame adaptCloud(const sensor_msgs::msg::PointCloud2 &m, tf2_ros::Buffer &tf,
                                const std::string &robot_frame, double received, size_t max_points)
{
  const size_t points = static_cast<size_t>(m.width) * m.height;
  if (m.header.frame_id.empty() || points > max_points || !m.point_step ||
      static_cast<size_t>(m.width) * m.point_step > m.row_step ||
      static_cast<size_t>(m.height) * m.row_step != m.data.size() || m.is_bigendian)
    throw std::invalid_argument("Invalid/excessive PointCloud2 layout");
  for (const std::string field : {"x", "y", "z"})
  {
    auto it = std::find_if(m.fields.begin(), m.fields.end(),
                           [&](const auto &f) { return f.name == field; });
    if (it == m.fields.end() || it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        it->count != 1 || static_cast<uint64_t>(it->offset) + 4 > m.point_step)
      throw std::invalid_argument("Cloud needs float32 XYZ");
  }
  const rclcpp::Time stamp(m.header.stamp, RCL_ROS_TIME);
  // All transforms use this observation's stamp. Every accumulated frame is already in map.
  if (stamp.nanoseconds() <= 0)
    throw std::invalid_argument("Cloud needs a nonzero measurement timestamp");
  Eigen::Isometry3d map_cloud = Eigen::Isometry3d::Identity();
  if (m.header.frame_id != "map")
    map_cloud = eigen(tf.lookupTransform("map", m.header.frame_id, stamp));
  auto robot = eigen(tf.lookupTransform("map", robot_frame, stamp));
  auto map_odom = eigen(tf.lookupTransform("map", "odom", stamp));
  DynamicCloud in;
  pcl::fromROSMsg(m, in);
  auto out = std::make_shared<DynamicCloud>();
  out->reserve(in.size());
  for (const auto &p : in)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    const Eigen::Vector3d v = map_cloud * p.getVector3fMap().cast<double>();
    out->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }
  if (points && out->empty())
    throw std::invalid_argument("No finite cloud measurements");
  return {"map", stamp.seconds(), received, out, robot.translation(), map_odom};
}
} // namespace map_server
