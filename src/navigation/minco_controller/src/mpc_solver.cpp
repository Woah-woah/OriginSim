// Adapted from Walker152/navi_minco_bit, commit 8288b7520ecb635bb27f16b0d49e3cf304f37a72.
#include "minco_controller/mpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "color_text.hpp"
#include <qpOASES.hpp>

namespace minco_controller {

MpcSolver::MpcSolver(const MPCConfig & config)
{
  setConfig(config);
}

void MpcSolver::setConfig(const MPCConfig & config)
{
  if (!std::isfinite(config.dt) || config.dt <= 0.0 ||
    config.horizon < 1 || config.horizon > 100 ||
    !config.Q.allFinite() || (config.Q.array() < 0.0).any() ||
    !config.control_delta_weight.allFinite() || (config.control_delta_weight.array() < 0.0).any() ||
    !config.R.allFinite() || (config.R.array() <= 0.0).any() ||
    !std::isfinite(config.q_along) || config.q_along < 0.0 ||
    !std::isfinite(config.q_cross) || config.q_cross < 0.0 ||
    !std::isfinite(config.eps_regularization) || config.eps_regularization <= 0.0 ||
    config.max_working_set_recalculations <= 0 ||
    !std::isfinite(config.obstacle_weight) || config.obstacle_weight < 0.0 ||
    !std::isfinite(config.obstacle_trust_region) || config.obstacle_trust_region <= 0.0 ||
    !std::isfinite(config.control_period) || config.control_period <= 0.0 ||
    !std::isfinite(config.safety_preview_time) || config.safety_preview_time < 0.0 ||
    config.obstacle_iterations < 1 || config.obstacle_iterations > 10)
  {throw std::invalid_argument("局部控制的预测步长、权重或求解参数无效");}
  for (const auto & bounds : {std::pair{config.vx_min, config.vx_max},
    std::pair{config.vy_min, config.vy_max}, std::pair{config.omega_min, config.omega_max},
    std::pair{config.ax_min, config.ax_max}, std::pair{config.ay_min, config.ay_max},
    std::pair{config.alpha_min, config.alpha_max}})
  {
    if (!std::isfinite(bounds.first) || !std::isfinite(bounds.second) ||
      bounds.first > 0.0 || bounds.second < 0.0 || bounds.first >= bounds.second)
    {throw std::invalid_argument("局部控制的速度和加速度上下限必须有效，并包含零");}
  }
  config_ = config;
  reset();
}

void MpcSolver::reset()
{
  last_u_global_.setZero();
  has_last_u_ = false;
  last_status_ = 0;
}

// 日志专用说明，不参与求解分支或安全判断。
std::string MpcSolver::lastStatusMessage() const
{
  switch (last_status_) {
    case 0: return "求解成功";
    case -1: return "当前状态、参考轨迹或延迟预测数据无效";
    case -2: return "预测位置无法取得有效地图或障碍距离数据";
    case -3: return "未找到满足硬避障约束和安全包络的速度";
    case -4: return "输出指令模型或安全包络配置无效";
    case qpOASES::RET_MAX_NWSR_REACHED: return "达到求解迭代上限，尚未找到有效速度";
    case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
      return "当前约束无法同时满足，找不到兼顾速度限制、避障和安全包络的速度";
    default: return "求解器返回异常，未生成可执行速度";
  }
}

void MpcSolver::buildStepModel(
  double /*yaw_ref*/, Eigen::Matrix3d & A, Eigen::Matrix<double, 3, 3> & B) const
{
  A.setIdentity();

  B.setZero();
  // 纯全局坐标系控制，解耦 X, Y, Yaw
  B(0, 0) = config_.dt;
  B(1, 1) = config_.dt;
  B(2, 2) = config_.dt;
}

bool MpcSolver::buildCondensedQP(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  const std::vector<CostLinearization> & costs,
  Eigen::MatrixXd & H,
  Eigen::VectorXd & g,
  Eigen::VectorXd & lb,
  Eigen::VectorXd & ub,
  Eigen::MatrixXd & A_con,
  Eigen::VectorXd & lbA,
  Eigen::VectorXd & ubA)
{
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  // 构建堆叠模型 X = A_hat x0 + B_hat U
  // 其中nx，nu 分别为单步状态与控制量维度，nX，nU 为堆叠后维度
  const int nx = 3;
  const int nu = 3;
  const int nU = nu * N;
  const int nX = nx * N;

  // 堆叠状态：X = [x1..xN, y1..yN, yaw1..yawN]
  // X = [x1,y1,yaw1, x2,y2,yaw2, ..., xN,yN,yawN]^T
  // X = A_hat * x0 + B_hat * U
  Eigen::MatrixXd A_hat = Eigen::MatrixXd::Zero(nX, nx);
  Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(nX, nU);

  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

  for (int i = 0; i < N; ++i) {
    Eigen::Matrix<double, 3, 3> Bi;
    buildStepModel(ref_traj[i].heading_yaw, A, Bi);

    // A_hat block: A^{i+1}
    Eigen::Matrix3d A_power = Eigen::Matrix3d::Identity();
    for (int p = 0; p < i + 1; ++p) {
      A_power = A * A_power;
    }
    A_hat.block<3, 3>(i * nx, 0) = A_power;

    // B_hat: 影响 x_{i+1} 的所有 u_0..u_i
    for (int j = 0; j <= i; ++j) {
      // 由于 A=I，这里仍按通用形式写：A^{i-j} * B_j
      Eigen::Matrix3d A_ij = Eigen::Matrix3d::Identity();
      for (int p = 0; p < i - j; ++p) {
        A_ij = A * A_ij;
      }

      Eigen::Matrix<double, 3, 3> Bj;
      buildStepModel(ref_traj[j].heading_yaw, A, Bj);

      B_hat.block<3, 3>(i * nx, j * nu) = A_ij * Bj;
    }
  }

  // 参考状态堆叠 X_ref
  Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(nX);
  for (int i = 0; i < N; ++i) {
    X_ref(i * nx + 0) = ref_traj[i].pos.x();
    X_ref(i * nx + 1) = ref_traj[i].pos.y();
    X_ref(i * nx + 2) = ref_traj[i].heading_yaw;
  }

  // 参考控制堆叠 U_ref (直接使用全局速度)
  Eigen::VectorXd U_ref = Eigen::VectorXd::Zero(nU);
  for (int i = 0; i < N; ++i) {
    U_ref(i * nu + 0) = ref_traj[i].vel.x();
    U_ref(i * nu + 1) = ref_traj[i].vel.y();
    U_ref(i * nu + 2) = ref_traj[i].yaw_rate;
  }

  // Q_bar / R_bar (对角块)
  // P1: Rotate Q at each horizon step to decompose tracking penalty
  // into along-track (heading direction) and cross-track (perpendicular).
  // Higher cross-track weight enforces precision on curves/narrow passages.
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nX, nX);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nU, nU);
  for (int i = 0; i < N; ++i) {
    const double yaw_ref = ref_traj[i].path_tangent_yaw;
    const double c = std::cos(yaw_ref);
    const double s = std::sin(yaw_ref);
    // 路径切线 t=(c,s)，法向 n=(-s,c)：Q_xy=qa*t*t^T+qc*n*n^T。
    // 朝向跟踪仅使用独立的 heading_yaw，不参与此处平移权重旋转。
    const double qa = config_.q_along;
    const double qc = config_.q_cross;
    Q_bar(i * nx + 0, i * nx + 0) = qa * c * c + qc * s * s;
    Q_bar(i * nx + 0, i * nx + 1) = (qa - qc) * c * s;
    Q_bar(i * nx + 1, i * nx + 0) = (qa - qc) * c * s;
    Q_bar(i * nx + 1, i * nx + 1) = qa * s * s + qc * c * c;
    Q_bar(i * nx + 2, i * nx + 2) = config_.Q.z();  // yaw weight unchanged

    R_bar.block<3, 3>(i * nu, i * nu) = config_.R.asDiagonal();
  }

  // 保留原相位补偿的速度前馈倾向，但将最终速度直接作为 QP 变量。
  // 等价于以原始速度 R(+phase)*u 计算输入残差；位置预测始终使用实际 u。
  Eigen::Matrix3d phase = Eigen::Matrix3d::Identity();
  phase(0, 0) = phase(1, 1) = std::cos(command_model_.phase);
  phase(0, 1) = std::sin(command_model_.phase);
  phase(1, 0) = -std::sin(command_model_.phase);
  U_ref.head<3>() = phase * U_ref.head<3>().eval();
  R_bar.block<3, 3>(0, 0) = phase * config_.R.asDiagonal() * phase.transpose();

  // d = A_hat * x0 - X_ref
  Eigen::Vector3d x0(curr.x, curr.y, curr.yaw);
  const Eigen::VectorXd d = A_hat * x0 - X_ref;

  // H = 2*(B^T Q B + R)
  H = 2.0 * (B_hat.transpose() * Q_bar * B_hat + R_bar);

  // 数值正则（避免 Hessian 奇异）
  H += config_.eps_regularization * Eigen::MatrixXd::Identity(nU, nU);

  // g = 2*(B^T Q d - R U_ref)
  g = 2.0 * (B_hat.transpose() * Q_bar * d - R_bar * U_ref);

  // Smooth accepted commands in the QP. The anchor is frozen for every
  // sequential obstacle solve; an intermediate QP must not become u_previous.
  const Eigen::Vector3d previous = has_last_u_ ? last_u_global_ :
    Eigen::Vector3d(curr.vx, curr.vy, curr.omega);
  for (int k = 0; k < N; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      const int col = 3 * k + axis;
      const double weight = config_.control_delta_weight[axis];
      H(col, col) += 2.0 * weight;
      if (k == 0) {
        g(col) -= 2.0 * weight * previous[axis];
      } else {
        H(col - 3, col - 3) += 2.0 * weight;
        H(col, col - 3) -= 2.0 * weight;
        H(col - 3, col) -= 2.0 * weight;
      }
    }
  }

  // p_i = p0 + B_xy U. Squaring the affine cost residual adds a PSD
  // Gauss-Newton Hessian 2*w*a*a^T; no map queries occur inside the QP.
  for (int i = 0; i < static_cast<int>(costs.size()); ++i) {
    const auto & cost = costs[i];
    const Eigen::RowVectorXd a = cost.gradient.transpose() * B_hat.block(i * nx, 0, 2, nU);
    const double b = cost.cost + cost.gradient.dot(x0.head<2>() - cost.position);
    H.noalias() += 2.0 * config_.obstacle_weight * a.transpose() * a;
    g.noalias() += 2.0 * config_.obstacle_weight * b * a.transpose();
  }

  // 变量边界（速度约束，map/global 系）
  // Use -1e12 instead of -inf: qpOASES defines INFTY=1e20 internally.
  // IEEE -inf causes NaN in active-set arithmetic.
  lb = Eigen::VectorXd::Constant(nU, -1.0e12);
  ub = Eigen::VectorXd::Constant(nU, 1.0e12);
  for (int i = 0; i < N; ++i) {
    lb(i * nu + 0) = config_.vx_min;
    ub(i * nu + 0) = config_.vx_max;
    lb(i * nu + 1) = config_.vy_min;
    ub(i * nu + 1) = config_.vy_max;
    lb(i * nu + 2) = config_.omega_min;
    ub(i * nu + 2) = config_.omega_max;
    if (command_model_.fixed_angular_velocity) {
      lb(i * nu + 2) = ub(i * nu + 2) = command_model_.fixed_wz;
    }
    if (i == 0 && zero_first_translation_) {
      lb(0) = ub(0) = lb(1) = ub(1) = 0.0;
    }
  }

  // 加速度约束（可选）：a_min <= (v_k - v_{k-1})/dt <= a_max
  if (!config_.use_acc_constraints && costs.empty()) {
    A_con.resize(0, 0);
    lbA.resize(0);
    ubA.resize(0);
    return true;
  }

  const int nAcc = config_.use_acc_constraints ? 3 * N : 0;
  int nSafety = 0;
  for (const auto & cost : costs) {nSafety += static_cast<int>(cost.safety_planes.size());}
  const int nC = nAcc + 2 * static_cast<int>(costs.size()) + nSafety;
  A_con = Eigen::MatrixXd::Zero(nC, nU);
  lbA = Eigen::VectorXd::Zero(nC);
  ubA = Eigen::VectorXd::Zero(nC);

  const double last_vx = curr.vx;
  const double last_vy = curr.vy;
  const double last_w = curr.omega;

  // 约束形式：lbA <= A_con * U <= ubA
  // 这里 A_con * U 直接表示 (v_k - v_{k-1})，再用 bounds 限制到 [a_min*dt, a_max*dt]
  for (int k = 0; k < nAcc / 3; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      const int row = k * 3 + axis;
      const int col_curr = k * 3 + axis;
      A_con(row, col_curr) = 1.0;

      if (k == 0) {
        // v0 - v(-1)
        // v(-1) 使用当前时刻真实全局速度作为已知常数，转移到 bounds：
        // a_min*dt <= v0 - v_curr_global <= a_max*dt
        const double last = (axis == 0 ? last_vx : (axis == 1 ? last_vy : last_w));
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.control_period +
          last;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.control_period +
          last;
      } else {
        const int col_prev = (k - 1) * 3 + axis;
        A_con(row, col_prev) = -1.0;
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.dt;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.dt;
      }
    }
  }

  for (int i = 0; i < static_cast<int>(costs.size()); ++i) {
    const int row = nAcc + 2 * i;
    A_con.block(row, 0, 2, nU) = B_hat.block(i * nx, 0, 2, nU);
    lbA.segment<2>(row) = costs[i].position - x0.head<2>() -
      Eigen::Vector2d::Constant(config_.obstacle_trust_region);
    ubA.segment<2>(row) = costs[i].position - x0.head<2>() +
      Eigen::Vector2d::Constant(config_.obstacle_trust_region);
  }
  int row = nAcc + 2 * static_cast<int>(costs.size());
  for (int i = 0; i < static_cast<int>(costs.size()); ++i) {
    for (const auto & plane : costs[i].safety_planes) {
      A_con.row(row) = plane.normal.transpose() * B_hat.block(i * nx, 0, 2, nU);
      A_con.block<1, 2>(row, i * nu) += plane.preview_time * plane.normal.transpose();
      lbA(row) = plane.offset - plane.normal.dot(x0.head<2>() + plane.origin_offset);
      ubA(row) = 1e12;
      ++row;
    }
  }
  return true;
}

bool MpcSolver::solve(const State & curr,
  const std::vector<ReferencePoint> & ref_traj, Control & out_u,
  std::vector<State> * out_pred, const CostSampler & sample_cost, const State * observed,
  const EnvelopeSampler & sample_envelope, const CommandModel & command_model,
  const MotionSampler & sample_motion)
{
  out_u = {};
  if (out_pred) {out_pred->clear();}
  if (!std::isfinite(command_model.phase) || !std::isfinite(command_model.deadzone) ||
    command_model.deadzone < 0 || !std::isfinite(command_model.fixed_wz) ||
    (command_model.fixed_angular_velocity &&
    (command_model.fixed_wz < config_.omega_min || command_model.fixed_wz > config_.omega_max)) ||
    (sample_cost && (!sample_motion || (observed && !sample_envelope))))
  {last_status_ = -4; return false;}
  command_model_ = command_model;
  zero_first_translation_ = false;
  Control accepted;
  std::vector<State> prediction;
  if (!solveImpl(curr, ref_traj, accepted, &prediction, sample_cost, observed, sample_envelope, sample_motion)) {
    return false;
  }
  const double speed = std::hypot(accepted.vx, accepted.vy);
  if (speed > 1e-10 && speed < command_model_.deadzone) {
    // 死区必须进入 QP 后重新预测和检查，禁止求解完成后直接把速度改成零。
    zero_first_translation_ = true;
    if (!solveImpl(curr, ref_traj, accepted, &prediction, sample_cost, observed, sample_envelope, sample_motion)) {
      return false;
    }
  }
  out_u = accepted;
  last_u_global_ = Eigen::Vector3d(out_u.vx, out_u.vy, out_u.omega);
  has_last_u_ = true;
  if (out_pred) {*out_pred = std::move(prediction);}
  return true;
}

bool MpcSolver::solveImpl(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Control & out_u, std::vector<State> * out_pred, const CostSampler & sample_cost,
  const State * observed, const EnvelopeSampler & sample_envelope, const MotionSampler & sample_motion)
{
  out_u = Control{};
  if (out_pred) {out_pred->clear();}
  if (observed && (!Eigen::Matrix<double, 6, 1>(observed->x, observed->y, observed->yaw,
    observed->vx, observed->vy, observed->omega).allFinite() ||
    (Eigen::Vector2d(curr.x, curr.y) - Eigen::Vector2d(observed->x, observed->y) -
    config_.safety_preview_time * Eigen::Vector2d(observed->vx, observed->vy)).norm() > 1e-8)) {
    last_status_ = -1;
    return false;
  }
  const Eigen::Vector3d previous_command = has_last_u_ ? last_u_global_ :
    Eigen::Vector3d(curr.vx, curr.vy, curr.omega);
  Control best_control;
  std::vector<State> best;
  if (!solveOnce(curr, ref_traj, {}, best_control, &best)) {return false;}
  if (sample_cost) {
    // The planned corridor can have unavoidable inflation cost on both sides.
    // Penalize excess over its reference cost, otherwise entering a traversable
    // narrow door creates a forward potential barrier and the robot stalls.
    std::vector<double> reference_cost(config_.horizon, 0.0);
    for (int i = 0; i < config_.horizon; ++i) {
      CostLinearization sample;
      if (sample_cost(ref_traj[i].pos, sample) && std::isfinite(sample.cost) && sample.cost >= 0.0) {
        reference_cost[i] = sample.cost;
      }
    }
    // Accept only decreases of the actual sampled objective, preventing an
    // inaccurate local approximation from increasing obstacle/tracking cost.
    const auto evaluate = [&](const std::vector<State> & prediction,
      std::vector<CostLinearization> & costs, double & objective) {
        objective = 0.0;
        costs.clear();
        for (int i = 0; i < config_.horizon; ++i) {
          const auto & state = prediction[i + 1];
          const auto & ref = ref_traj[i];
          CostLinearization cost;
          cost.position = Eigen::Vector2d(state.x, state.y);
          if (!sample_cost(cost.position, cost) || !std::isfinite(cost.cost) ||
            cost.cost < 0.0 || !cost.gradient.allFinite()) {return false;}
          cost.position = Eigen::Vector2d(state.x, state.y);
          // 平面由这一候选解的真实轮廓朝向和完整移动过程重建，不使用外接圆。
          if (!sample_motion(prediction[i], state, cost.safety_planes)) {return false;}
          const double excess = cost.cost - reference_cost[i];
          cost.cost = std::max(0.0, excess);
          if (excess <= 0.0) {cost.gradient.setZero();}
          for (const auto & plane : cost.safety_planes) {
            if (!plane.normal.allFinite() || plane.normal.squaredNorm() < 1e-12 ||
              !std::isfinite(plane.offset) || !std::isfinite(plane.preview_time) ||
              !plane.origin_offset.allFinite()) {return false;}
          }
          costs.push_back(cost);
          const Eigen::Vector2d error = cost.position - ref.pos;
          const double along = std::cos(ref.path_tangent_yaw) * error.x() + std::sin(ref.path_tangent_yaw) * error.y();
          const double cross = -std::sin(ref.path_tangent_yaw) * error.x() + std::cos(ref.path_tangent_yaw) * error.y();
          Eigen::Vector3d actual(state.vx, state.vy, state.omega);
          if (i == 0) {
            const double c = std::cos(command_model_.phase), s = std::sin(command_model_.phase);
            actual.head<2>() = Eigen::Vector2d(c * state.vx - s * state.vy,
              s * state.vx + c * state.vy);
          }
          const Eigen::Vector3d du = actual - Eigen::Vector3d(ref.vel.x(), ref.vel.y(), ref.yaw_rate);
          const auto & prior = prediction[i];
          const Eigen::Vector3d delta_u = Eigen::Vector3d(state.vx, state.vy, state.omega) -
            (i == 0 ? previous_command : Eigen::Vector3d(prior.vx, prior.vy, prior.omega));
          objective += delta_u.dot(config_.control_delta_weight.asDiagonal() * delta_u);
          objective += config_.q_along * along * along + config_.q_cross * cross * cross +
            config_.Q.z() * std::pow(state.yaw - ref.heading_yaw, 2) +
            du.dot(config_.R.asDiagonal() * du) + config_.obstacle_weight * cost.cost * cost.cost;
        }
        if (observed) {
          std::vector<SafetyPlane> planes;
          const auto & first = prediction[1];
          const Control command{first.vx, first.vy, first.omega};
          if (!sample_envelope(*observed, command, planes)) {return false;}
          for (const auto & plane : planes) {
            if (!plane.normal.allFinite() || !std::isfinite(plane.offset) ||
              !std::isfinite(plane.preview_time) || !plane.origin_offset.allFinite() ||
              plane.normal.squaredNorm() < 1e-12) {return false;}
            costs.front().safety_planes.push_back(plane);
          }
        }
        return std::isfinite(objective);
      };
    const auto safe = [&](const std::vector<State> & prediction,
      const std::vector<CostLinearization> & costs) {
        for (int i = 0; i < config_.horizon; ++i) {
          const auto & state = prediction[i + 1];
          const Eigen::Vector2d position(state.x, state.y);
          for (const auto & plane : costs[i].safety_planes) {
            const Eigen::Vector2d point = position +
              plane.preview_time * Eigen::Vector2d(state.vx, state.vy) + plane.origin_offset;
            if (plane.normal.dot(point) < plane.offset - 1e-7) {return false;}
          }
        }
        return true;
      };
    std::vector<CostLinearization> costs;
    double best_value;
    if (!evaluate(best, costs, best_value)) {last_status_ = -2; return false;}
    for (int iteration = 0; iteration < config_.obstacle_iterations; ++iteration) {
      Control candidate_control;
      std::vector<State> candidate;
      const bool best_is_safe = safe(best, costs);
      // 基础 QP 已是跟踪/速度目标的最优解；若完整轮廓检查安全且障碍残差为零，
      // 非负障碍代价不可能再改善目标。直接保留已认证解，避免重复求解和轮廓生成。
      if (iteration == 0 && best_is_safe && std::all_of(costs.begin(), costs.end(),
        [](const auto & cost) {return cost.cost == 0.0;})) {break;}
      if (!solveOnce(curr, ref_traj, costs, candidate_control, &candidate)) {
        if (best_is_safe) {last_status_ = 0; break;}
        return false;
      }
      std::vector<CostLinearization> candidate_costs;
      double value;
      if (!evaluate(candidate, candidate_costs, value)) {
        if (best_is_safe) {last_status_ = 0; break;}
        last_status_ = -2;
        return false;
      }
      // Hard clearance takes priority over the soft objective. Never retain
      // an unsafe tracking-only seed merely because making it safe costs more.
      if (best_is_safe && (!safe(candidate, candidate_costs) || value >= best_value)) {break;}
      best_value = value;
      best = std::move(candidate);
      costs = std::move(candidate_costs);
      best_control = candidate_control;
    }
    if (!safe(best, costs)) {last_status_ = -3; return false;}
  }
  out_u = best_control;
  if (out_pred) {*out_pred = std::move(best);}
  return true;
}

bool MpcSolver::solveOnce(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  const std::vector<CostLinearization> & costs,
  Control & out_u, std::vector<State> * out_pred)
{
  out_u = Control{};
  if (out_pred) {out_pred->clear();}
  last_status_ = -1;
  if (!Eigen::Matrix<double, 6, 1>(curr.x, curr.y, curr.yaw,
      curr.vx, curr.vy, curr.omega).allFinite()) {return false;}
  for (const auto & point : ref_traj) {
    if (!point.pos.allFinite() || !point.vel.allFinite() ||
      !std::isfinite(point.heading_yaw) || !std::isfinite(point.path_tangent_yaw) ||
      !std::isfinite(point.yaw_rate)) {return false;}
  }
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::MatrixXd A_con;
  Eigen::VectorXd lbA;
  Eigen::VectorXd ubA;

  if (!buildCondensedQP(curr, ref_traj, costs, H, g, lb, ub, A_con, lbA, ubA)) {
    return false;
  }

  const int nV = static_cast<int>(g.size());
  const int nC = static_cast<int>(lbA.size());

  std::vector<qpOASES::real_t> H_qp(static_cast<size_t>(nV) * static_cast<size_t>(nV), 0.0);
  for (int i = 0; i < nV; ++i) {
    for (int j = 0; j < nV; ++j) {
      H_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
        static_cast<qpOASES::real_t>(H(i, j));
    }
  }

  std::vector<qpOASES::real_t> g_qp(static_cast<size_t>(nV), 0.0);
  std::vector<qpOASES::real_t> lb_qp(static_cast<size_t>(nV), 0.0);
  std::vector<qpOASES::real_t> ub_qp(static_cast<size_t>(nV), 0.0);
  for (int i = 0; i < nV; ++i) {
    g_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(g(i));
    lb_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lb(i));
    ub_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ub(i));
  }

  std::vector<qpOASES::real_t> A_qp;
  std::vector<qpOASES::real_t> lbA_qp;
  std::vector<qpOASES::real_t> ubA_qp;

  const qpOASES::real_t * A_ptr = nullptr;
  const qpOASES::real_t * lbA_ptr = nullptr;
  const qpOASES::real_t * ubA_ptr = nullptr;

  if (nC > 0) {
    A_qp.assign(static_cast<size_t>(nC) * static_cast<size_t>(nV), 0.0);
    for (int i = 0; i < nC; ++i) {
      for (int j = 0; j < nV; ++j) {
        A_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
          static_cast<qpOASES::real_t>(A_con(i, j));
      }
    }

    lbA_qp.assign(static_cast<size_t>(nC), 0.0);
    ubA_qp.assign(static_cast<size_t>(nC), 0.0);
    for (int i = 0; i < nC; ++i) {
      lbA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lbA(i));
      ubA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ubA(i));
    }

    A_ptr = A_qp.data();
    lbA_ptr = lbA_qp.data();
    ubA_ptr = ubA_qp.data();
  }

  qpOASES::QProblem qp(nV, nC);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);

  qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(config_.max_working_set_recalculations);
  const qpOASES::returnValue ret =
    qp.init(H_qp.data(), g_qp.data(), A_ptr, lb_qp.data(), ub_qp.data(), lbA_ptr, ubA_ptr, nWSR);

  last_status_ = static_cast<int>(ret);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      std::cout << color_text::RED << "[局部控制求解器] 达到允许的求解迭代次数，尚未找到有效解" << color_text::RESET
                << std::endl;
    } else if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
      std::cout << color_text::RED << "[局部控制求解器] 当前约束无法同时满足，未找到可执行速度" << color_text::RESET << std::endl;
    } else {
      std::cout << color_text::RED << "[局部控制求解器] 求解器返回异常，状态码=" << static_cast<int>(ret)
                << color_text::RESET << std::endl;
    }
    return false;
  }

  std::vector<qpOASES::real_t> xOpt(static_cast<size_t>(nV), 0.0);
  if (qp.getPrimalSolution(xOpt.data()) != qpOASES::SUCCESSFUL_RETURN ||
    !std::all_of(xOpt.begin(), xOpt.end(), [](auto value) {return std::isfinite(value);}))
  {return false;}

  // 如果需要输出预测轨迹
  if (out_pred) {
    out_pred->clear();
    out_pred->reserve(N + 1);
    out_pred->push_back(curr);

    State s = curr;
    Eigen::Matrix3d A;  // Identity inside buildStepModel
    Eigen::Matrix<double, 3, 3> B;

    for (int i = 0; i < N; ++i) {
      const double ux = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 0)]);
      const double uy = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 1)]);
      const double uw = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 2)]);

      // 使用 QP 构建时的线性化点 (reference yaw) 进行推演
      buildStepModel(ref_traj[i].heading_yaw, A, B);

      double dx = B(0, 0) * ux + B(0, 1) * uy + B(0, 2) * uw;
      double dy = B(1, 0) * ux + B(1, 1) * uy + B(1, 2) * uw;
      double dyaw = B(2, 0) * ux + B(2, 1) * uy + B(2, 2) * uw;

      s.x += dx;
      s.y += dy;
      s.yaw += dyaw;
      s.vx = ux;
      s.vy = uy;
      s.omega = uw;
      out_pred->push_back(s);
    }
  }

  // 取第一个控制量 u0 (global)
  const Eigen::Vector3d u0_global(
    static_cast<double>(xOpt[0]), static_cast<double>(xOpt[1]), static_cast<double>(xOpt[2]));

  // 直接输出全局速度
  out_u.vx = u0_global.x();
  out_u.vy = u0_global.y();
  out_u.omega = u0_global.z();

  return true;
}

}  // namespace minco_controller
