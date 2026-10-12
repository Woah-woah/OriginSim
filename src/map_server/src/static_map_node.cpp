#include "map_server/static_map_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"

namespace map_server
{
class StaticMapNode : public rclcpp::Node
{
public:
  StaticMapNode() : Node("map_server")
  {
    map_ = loadStaticMap(declare_parameter("yaml_filename", std::string{}));
    map_.header.frame_id = declare_parameter("frame_id", std::string("map"));
    if (map_.header.frame_id.empty()) {throw std::invalid_argument("Map frame_id is empty");}
    map_.header.stamp = now(); map_.info.map_load_time = now();
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/map")), rclcpp::QoS(1).reliable().transient_local());
    publisher_->publish(map_);
    service_ = create_service<nav_msgs::srv::GetMap>("map_server/map",
      [this](nav_msgs::srv::GetMap::Request::SharedPtr, nav_msgs::srv::GetMap::Response::SharedPtr out) {
        out->map = map_;
      });
    RCLCPP_INFO(get_logger(), "Published occupancy map %u x %u, resolution %.3f",
      map_.info.width, map_.info.height, map_.info.resolution);
  }
private:
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr service_;
};
}  // namespace map_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<map_server::StaticMapNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("map_server"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
