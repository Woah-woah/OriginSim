#pragma once

#include "mppi_controller/controller.hpp"
#include "nav_executor/controller_plugin.hpp"

namespace mppi_controller
{
class MppiControllerPlugin : public nav_executor::ControllerPlugin
{
public:
  void initialize(
    const rclcpp::Node::WeakPtr & node, const std::string & name,
    const std::shared_ptr<map_server::CostmapClient> & costmap) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void reset() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & measured_speed,
    const nav_executor::GoalTolerances & tolerances) override;

private:
  MPPIController controller_;
};
}  // namespace mppi_controller
