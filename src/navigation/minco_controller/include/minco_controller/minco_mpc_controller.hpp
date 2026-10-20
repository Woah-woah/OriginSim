#pragma once

#include <memory>
#include <string>

#include "minco_controller/mpc_solver.hpp"
#include "minco_controller/path_trajectory.hpp"
#include "nav_executor/controller_plugin.hpp"

namespace minco_controller
{
class MincoMpcController : public nav_executor::ControllerPlugin
{
public:
  void initialize(const rclcpp::Node::WeakPtr & node, const std::string & name,
    const std::shared_ptr<map_server::CostmapClient> & costmap) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void reset() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & measured_speed,
    const nav_executor::GoalTolerances & tolerances) override;

private:
  void checkPrediction(const std::vector<State> & prediction, const std::string & stage) const;
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<map_server::CostmapClient> costmap_;
  MPCConfig config_;
  std::unique_ptr<MpcSolver> solver_;
  PathTrajectory trajectory_;
  std::string plan_frame_;
  double reference_speed_{0.6}, reference_acceleration_{1.0}, reference_yaw_rate_{1.0};
  double control_delay_{0.25}, output_delay_{0.025};
  double deadzone_{0.02}, velocity_noise_threshold_{0.03}, fixed_wz_{0.0};
  double obstacle_smoothing_distance_{0.2};
  double progress_time_{0.0}, collision_margin_{0.02};
  bool small_gyro_mode_{true};
};
}  // namespace minco_controller
