#include "mppi_controller/controller_plugin.hpp"

#include <stdexcept>

#include "map_server/costmap_client.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi_controller
{
namespace
{
class GoalCheckerAdapter : public mppi::GoalChecker
{
public:
  explicit GoalCheckerAdapter(const nav_executor::GoalTolerances & tolerances)
  : tolerances_(tolerances) {}

  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose.position.x = pose.position.y = tolerances_.xy;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, tolerances_.yaw);
    pose.orientation = tf2::toMsg(rotation);
    velocity.linear.x = velocity.linear.y = velocity.angular.z = tolerances_.stopped_speed;
    return true;
  }

private:
  const nav_executor::GoalTolerances & tolerances_;
};
}  // namespace

void MppiControllerPlugin::initialize(
  const rclcpp::Node::WeakPtr & parent, const std::string & name,
  const std::shared_ptr<map_server::CostmapClient> & costmap)
{
  const auto node = parent.lock();
  if (!node || !costmap) {
    throw std::invalid_argument("MPPI plugin requires a node and CostmapClient");
  }
  // Keep measured odometry feedback mandatory, as in the original executor.
  const auto open_loop = name + ".open_loop";
  if (!node->has_parameter(open_loop)) {node->declare_parameter(open_loop, false);}
  if (node->get_parameter(open_loop).as_bool()) {
    throw std::invalid_argument(open_loop + " must be false for measured odometry feedback");
  }
  mppi::CostmapConfig config;
  config.global_frame = costmap->getGlobalFrameID();
  config.robot_base_frame = costmap->getBaseFrameID();
  config.footprint = costmap->getRobotFootprint();
  config.robot_radius = costmap->settings().robot_radius;
  config.track_unknown = costmap->settings().track_unknown;
  config.inflation_radius = costmap->settings().inflation_radius;
  config.inflation_cost_scaling_factor = costmap->settings().cost_scaling_factor;
  controller_.initialize(parent, name, costmap->getTfBuffer(), costmap->getCostmap(), config);
}

void MppiControllerPlugin::setPlan(const nav_msgs::msg::Path & path)
{controller_.setPlan(path);}

void MppiControllerPlugin::reset()
{controller_.reset();}

geometry_msgs::msg::TwistStamped MppiControllerPlugin::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & measured_speed,
  const nav_executor::GoalTolerances & tolerances)
{
  GoalCheckerAdapter goal_checker(tolerances);
  return controller_.computeVelocityCommands(pose, measured_speed, &goal_checker);
}
}  // namespace mppi_controller

PLUGINLIB_EXPORT_CLASS(mppi_controller::MppiControllerPlugin, nav_executor::ControllerPlugin)
