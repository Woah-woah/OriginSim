#include "map_server/map_geometry.hpp"
#include <yaml-cpp/yaml.h>

namespace map_server
{
MapSettings declareMapSettings(rclcpp::Node &node)
{
  // ---------------------参数声明-------------------------
  MapSettings settings;
  settings.global_frame = node.declare_parameter("global_frame", std::string("map"));
  settings.robot_base_frame = node.declare_parameter("robot_base_frame", std::string("base_link_fake"));
  settings.footprint_frame = node.declare_parameter("footprint_frame", std::string("base_link_fake"));
  settings.robot_radius = node.declare_parameter("robot_radius", 0.15);
  settings.inflation_radius = node.declare_parameter("inflation_radius", 0.55);
  settings.cost_scaling_factor = node.declare_parameter("cost_scaling_factor", 10.0);
  settings.track_unknown = node.declare_parameter("track_unknown_space", true);
  const auto text = node.declare_parameter("footprint", std::string{});

  if (settings.global_frame != "map" || settings.robot_base_frame.empty() ||
      settings.footprint_frame.empty() || !std::isfinite(settings.robot_radius) ||
      settings.robot_radius <= 0 || !std::isfinite(settings.inflation_radius) ||
      settings.inflation_radius < 0 || !std::isfinite(settings.cost_scaling_factor) ||
      settings.cost_scaling_factor < 0)
  {
    throw std::invalid_argument("Invalid costmap frame, robot geometry or inflation parameters");
  }

  // 参考nav2设计：如果没有写footprint顶点，就根据 radius 自动生成 32 边形脚点
  if (!text.empty())
  {
    const auto polygon = YAML::Load(text);
    
    if (!polygon.IsSequence() || polygon.size() < 3)
    {
      throw std::invalid_argument("Footprint requires at least three [x,y] vertices");
    }

    for (const auto &vertex : polygon)
    {
      if (!vertex.IsSequence() || vertex.size() != 2)
      {
        throw std::invalid_argument("Footprint vertex must be [x,y]");
      }

      geometry_msgs::msg::Point p;

      p.x = vertex[0].as<double>();
      p.y = vertex[1].as<double>();

      if (!std::isfinite(p.x) || !std::isfinite(p.y))
      {
        throw std::invalid_argument("Footprint coordinates must be finite");
      }

      settings.footprint.push_back(p);
    }

    if (!pointInPolygon(0.0, 0.0, settings.footprint) ||
        calculateMinAndMaxDistances(settings.footprint).first <= 0)
    {
      throw std::invalid_argument("Footprint must enclose the robot origin");
    }
  }
  else
  {
    for (int i = 0; i < 32; ++i)
    {
      geometry_msgs::msg::Point p;
      const double angle = i * 2 * std::acos(-1.0) / 32;

      p.x = settings.robot_radius * std::cos(angle);
      p.y = settings.robot_radius * std::sin(angle);

      settings.footprint.push_back(p);
    }
  }
  return settings;
}
} // namespace map_server
