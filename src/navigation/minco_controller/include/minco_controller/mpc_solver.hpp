#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>

#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// 纯 C++ MPC 求解器：不依赖 ROS，仅依赖 Eigen 与 qpOASES。
class MpcSolver
{
public:
  // Evaluated only between QP solves. False means unavailable/invalid map data.
  using CostSampler = std::function<bool(const Eigen::Vector2d &, CostLinearization &)>;

  explicit MpcSolver(const MPCConfig & config);

  void setConfig(const MPCConfig & config);

  // @brief 求解一次 MPC。
  // @param curr 当前状态 (map)
  // @param ref_traj 参考轨迹 (horizon 个点，若不足由插件层补齐)
  // @param out_u 输出控制 (map)
  // @param out_pred (可选) 输出预测轨迹
  // @param sample_cost Cost and lethal-cell supporting planes, evaluated outside QP
  // @param observed Undelayed pose anchoring first-command hold safety constraints
  // @return 是否求解成功
  bool solve(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Control & out_u,
    std::vector<State> * out_pred = nullptr,
    const CostSampler & sample_cost = {},
    const State * observed = nullptr);

  void reset();
  int lastStatus() const {return last_status_;}

  Eigen::Vector3d getLastControl() const { return last_u_global_; }

private:
  bool solveOnce(const State & curr, const std::vector<ReferencePoint> & ref_traj,
    const std::vector<CostLinearization> & costs, Control & out_u,
    std::vector<State> * out_pred);

  // 构建离散线性模型：x_{k+1} = A x_k + B_k u_k
  // 状态为 [x,y,yaw]，控制变量 u_k 为 map/global 系 [vx, vy, omega]
  void buildStepModel(double yaw_ref, Eigen::Matrix3d & A, Eigen::Matrix<double, 3, 3> & B) const;

  // 组装 MPC 的 condensed QP：0.5*U^T H U + g^T U
  // 其中 U 堆叠了 horizon 个控制量 [vx, vy, omega] (map/global)
  bool buildCondensedQP(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    const std::vector<CostLinearization> & costs,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & g,
    Eigen::VectorXd & lb,
    Eigen::VectorXd & ub,
    Eigen::MatrixXd & A_con,
    Eigen::VectorXd & lbA,
    Eigen::VectorXd & ubA);

private:
  MPCConfig config_;

  // Previous accepted global command; frozen across sequential QP iterations.
  Eigen::Vector3d last_u_global_{0.0, 0.0, 0.0};
  bool has_last_u_{false};
  int last_status_{0};
};

}  // namespace minco_controller
