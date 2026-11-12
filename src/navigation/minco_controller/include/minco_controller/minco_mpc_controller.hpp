#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include <unordered_set>

#include "minco_controller/mpc_solver.hpp"
#include "minco_controller/path_trajectory.hpp"
#include "minco_controller/trajectory_safety.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/node.hpp"
#include "map_server/footprint.hpp"

namespace map_server {class MapClient; class MapView;}

namespace minco_controller
{
class MapChanged : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class ControllerInfeasible : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// 普通导航模式下，MPC 预测运动存在碰撞风险
class PredictedCollision : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class MincoMpcController
{
public:
  void initialize(
    const rclcpp::Node::WeakPtr & node, const std::string & name,
    const std::shared_ptr<map_server::MapClient> & costmap
  );
  void setPlan(
    const nav_msgs::msg::Path & path,
    const double recovery_speed = 0,
    const double recovery_acceleration = 0
  );
  // 必须在停稳、设置 Recovery 路径后传入冻结的初始接触状态。
  void setRecoveryContacts(const RecoveryContacts & contacts);
  void reset();
  void notifyZeroCommandPublished();
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & measured_speed
  );

private:
  void checkPrediction(
    const std::vector<State> & prediction,
    const std::string & stage,
    const TrajectorySafety & field
  ) const;

  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<map_server::MapClient> costmap_;
  MPCConfig config_;
  std::unique_ptr<MpcSolver> solver_;
  PathTrajectory trajectory_;
  std::string plan_frame_;
  double reference_speed_{0.6}, reference_acceleration_{1.0}, reference_yaw_rate_{1.0};
  double control_delay_{0.10}, output_delay_{0.025};
  double deadzone_{0.02}, velocity_noise_threshold_{0.03}, fixed_wz_{0.0};
  double progress_time_{0.0}, collision_margin_{0.02};
  bool small_gyro_mode_{true};
  bool recovery_mode_{false};
  RecoveryContacts recovery_contacts_;
  double recovery_acceleration_{0.0};
};
}  // namespace minco_controller
