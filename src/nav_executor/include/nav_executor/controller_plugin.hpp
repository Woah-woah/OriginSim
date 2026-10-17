#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/node.hpp"

namespace map_server {class CostmapClient;}

namespace nav_executor
{
struct GoalTolerances
{
  double xy{0.3};
  double yaw{3.14};
  double stopped_speed{0.05};
};

// Called serially by PathExecutor. The host owns TF, costmap, odometry,
// goal completion, safety checks and command publication. Plugins only compute.
class ControllerPlugin
{
public:
  virtual ~ControllerPlugin() = default;
  // name is the plugin instance's parameter namespace. The host keeps the
  // costmap alive until after all plugin instances have been destroyed.
  virtual void initialize(
    const rclcpp::Node::WeakPtr & node, const std::string & name,
    const std::shared_ptr<map_server::CostmapClient> & costmap) = 0;
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
  virtual void reset() = 0;
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & measured_speed,
    const GoalTolerances & tolerances) = 0;
};
}  // namespace nav_executor
