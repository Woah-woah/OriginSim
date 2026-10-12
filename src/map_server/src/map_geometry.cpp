#include "map_server/map_geometry.hpp"
#include <yaml-cpp/yaml.h>

namespace map_server
{
MapSettings declareMapSettings(rclcpp::Node & node)
{
  MapSettings settings;
  settings.global_frame = node.declare_parameter("global_frame", std::string("map"));
  settings.robot_base_frame = node.declare_parameter("robot_base_frame", std::string("base_link_fake"));
  settings.robot_radius = node.declare_parameter("robot_radius", 0.15);
  settings.inflation_radius = node.declare_parameter("inflation_radius", 0.55);
  settings.cost_scaling_factor = node.declare_parameter("cost_scaling_factor", 10.0);
  settings.track_unknown = node.declare_parameter("track_unknown_space", true);
  const auto text = node.declare_parameter("footprint", std::string{});
  const double padding = node.declare_parameter("footprint_padding", 0.01);
  if (settings.global_frame.empty() || settings.robot_base_frame.empty() ||
    !std::isfinite(settings.robot_radius) || settings.robot_radius <= 0 ||
    !std::isfinite(settings.inflation_radius) || settings.inflation_radius < 0 ||
    !std::isfinite(settings.cost_scaling_factor) || settings.cost_scaling_factor < 0 ||
    !std::isfinite(padding) || padding < 0)
  {throw std::invalid_argument("Invalid costmap frame, robot geometry or inflation parameters");}
  if (!text.empty()) {
    const auto polygon = YAML::Load(text);
    if (!polygon.IsSequence() || polygon.size() < 3) {
      throw std::invalid_argument("Footprint requires at least three [x,y] vertices");
    }
    for (const auto & vertex : polygon) {
      if (!vertex.IsSequence() || vertex.size() != 2) {
        throw std::invalid_argument("Footprint vertex must be [x,y]");
      }
      geometry_msgs::msg::Point p;
      p.x = vertex[0].as<double>(); p.y = vertex[1].as<double>();
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        throw std::invalid_argument("Footprint coordinates must be finite");
      }
      p.x += p.x > 0 ? padding : p.x < 0 ? -padding : 0;
      p.y += p.y > 0 ? padding : p.y < 0 ? -padding : 0;
      settings.footprint.push_back(p);
    }
    if (!pointInPolygon(0.0, 0.0, settings.footprint) ||
      calculateMinAndMaxDistances(settings.footprint).first <= 0)
    {throw std::invalid_argument("Footprint must enclose the robot origin");}
  } else {
    for (int i = 0; i < 16; ++i) {
      geometry_msgs::msg::Point p;
      const double angle = i * 2 * std::acos(-1.0) / 16;
      p.x = settings.robot_radius * std::cos(angle);
      p.y = settings.robot_radius * std::sin(angle);
      p.x += p.x > 0 ? padding : p.x < 0 ? -padding : 0;
      p.y += p.y > 0 ? padding : p.y < 0 ? -padding : 0;
      settings.footprint.push_back(p);
    }
  }
  return settings;
}
}  // namespace map_server
