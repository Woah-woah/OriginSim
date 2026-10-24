#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>

namespace minco_controller {

// 状态：全局系 (map) 下的位姿
struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

// 控制量：速度命令 (vx, vy, omega)
// 约定：对外输出为全局系 (map) 下速度；插件层再转换到 base_link。
struct Control
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

// 参考点：来自 Minco 轨迹的高精度前馈信息
struct ReferencePoint
{
  Eigen::Vector2d pos{0.0, 0.0};  // map系位置
  Eigen::Vector2d vel{0.0, 0.0};  // map系速度(前馈)
  double heading_yaw{0.0};        // 车体朝向参考，独立于平移方向
  double path_tangent_yaw{0.0};   // 路径几何切线，用于沿程/横向误差
  double yaw_rate{0.0};           // 参考角速度(前馈)
};

// QP 变量直接表示最终发送的全局速度；相位补偿在速度前馈目标内建模。
struct CommandModel
{
  double phase{0.0};
  double deadzone{0.0};
  bool fixed_angular_velocity{false};
  double fixed_wz{0.0};
};

// 有朝向的 Footprint 与原始障碍栅格之间的分离平面。
struct SafetyPlane
{
  Eigen::Vector2d normal{0.0, 0.0};
  double offset{0.0};  // normal.dot(position) >= offset
  // Applied to p_i + preview_time*u_i + origin_offset (global XY).
  double preview_time{0.0};
  Eigen::Vector2d origin_offset{0.0, 0.0};
};

// C(p) ~= cost + gradient.dot(p - position), in the Costmap global frame.
struct CostLinearization
{
  Eigen::Vector2d position{0.0, 0.0};
  double cost{0.0};
  Eigen::Vector2d gradient{0.0, 0.0};
  std::vector<SafetyPlane> safety_planes;
};

// MPC 配置
struct MPCConfig
{
  // 采样周期与预测步长
  double dt{0.05};
  double planner_freq{20.0};
  double lookahead_time{0.5};

  int horizon{10};

  // 代价权重：Q(状态误差) / R(控制误差)
  // 状态采用 [x, y, yaw]，控制采用 [vx, vy, omega]
  // Q_along/cross 随路径切线旋转，不使用车体朝向。
  Eigen::Vector3d Q{5.0, 5.0, 2.0};
  double q_along{3.0};   // weight along heading (lower → timing flexibility)
  double q_cross{12.0};  // weight cross-track (higher → precision)
  Eigen::Vector3d R{1.0, 1.0, 0.5};
  // PSD penalty on u0 - previous accepted command and uk - u(k-1).
  Eigen::Vector3d control_delta_weight{2.0, 2.0, 0.5};

  // 速度约束（作用在 map/global 速度变量上）
  double vx_min{-1.0};
  double vx_max{1.0};
  double vy_min{-1.0};
  double vy_max{1.0};
  double omega_min{-2.0};
  double omega_max{2.0};

  // 加速度约束（可选）
  bool use_acc_constraints{false};
  double ax_min{-2.0};
  double ax_max{2.0};
  double ay_min{-2.0};
  double ay_max{2.0};
  double alpha_min{-4.0};
  double alpha_max{4.0};

  // 归一化 EDT 距离代价的平方残差，在 qpOASES 外线性化。
  double obstacle_weight{30.0};
  double obstacle_trust_region{0.15};  // metres, per predicted x/y
  int obstacle_iterations{3};
  // 未标定执行延迟的保守上界，和控制器的名义延迟共用同一配置。
  double safety_preview_time{0.0};
  double control_period{0.05};

  // qpOASES 求解器参数
  int max_working_set_recalculations{200};
  int max_iterations{200};
  double eps_regularization{1e-9};
};

}  // namespace minco_controller
