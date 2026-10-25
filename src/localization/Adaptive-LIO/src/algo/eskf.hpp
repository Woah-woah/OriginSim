//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_ESKF_HPP
#define SLAM_IN_AUTO_DRIVING_ESKF_HPP

#include "common/eigen_types.h"
#include "tools/imu.h"
#include "algo/lidar_observation.h"
// Core filter deliberately has no ROS, OpenCV or logging dependency.
#include "tools/nav_state.h"

#include <cmath>
#include <limits>

namespace zjloc
{

    /**
     * 书本第3章介绍的误差卡尔曼滤波器
     *
     * 本书使用18维的ESKF，标量类型可以由S指定，默认取double
     * 变量顺序：p, v, R, bg, ba, grav，与书本对应
     * @tparam S    状态变量的精度，取float或double
     */
    template <typename S = double>
    class ESKF
    {
    public:
        /// 类型定义
        using SO3 = Sophus::SO3<S>;                    // 旋转变量类型
        using VecT = Eigen::Matrix<S, 3, 1>;           // 向量类型
        using Vec18T = Eigen::Matrix<S, 18, 1>;        // 18维向量类型
        using Mat3T = Eigen::Matrix<S, 3, 3>;          // 3x3矩阵类型
        using MotionNoiseT = Eigen::Matrix<S, 18, 18>; // 运动噪声类型
        using Mat18T = Eigen::Matrix<S, 18, 18>;       // 18维方差类型
        using NavStateT = NavState<S>;                 // 整体名义状态变量类型

        struct Options
        {
            Options() = default;

            /// IMU 测量与零偏参数
            double imu_dt_ = 0.01; // IMU测量间隔
            // Legacy discrete covariance increments at imu_dt_; scale by dt/imu_dt_.
            // Values are variances/increments, NOT standard deviations or calibrated PSD.
            double gyro_var_ = 1e-5;      // 姿态离散协方差增量
            double acce_var_ = 1e-2;      // 速度离散协方差增量
            double bias_gyro_var_ = 1e-6; // 陀螺零偏离散协方差增量
            double bias_acce_var_ = 1e-4; // 加计零偏离散协方差增量

            /// 其他配置
            bool update_bias_gyro_ = true; // 是否更新陀螺bias
            bool update_bias_acce_ = true; // 是否更新加计bias
        };

        struct Snapshot {
            NavStateT nav;
            VecT gravity;
            Mat18T covariance;
            Options options;
            MotionNoiseT process_noise;
        };

        Snapshot ExportSnapshot() const {
            return {GetNominalState(), g_, cov_, options_, Q_};
        }

        static bool ValidSnapshot(const Snapshot& s) {
            const auto& n = s.nav;
            if (!std::isfinite(n.timestamp_) || !n.p_.allFinite() || !n.v_.allFinite() ||
                !n.bg_.allFinite() || !n.ba_.allFinite() || !s.gravity.allFinite() ||
                !n.R_.matrix().allFinite() || !s.covariance.allFinite() ||
                !s.process_noise.allFinite() || !std::isfinite(s.options.imu_dt_) ||
                s.options.imu_dt_ <= 0 ||
                !std::isfinite(s.options.gyro_var_) || s.options.gyro_var_ < 0 ||
                !std::isfinite(s.options.acce_var_) || s.options.acce_var_ < 0 ||
                !std::isfinite(s.options.bias_gyro_var_) || s.options.bias_gyro_var_ < 0 ||
                !std::isfinite(s.options.bias_acce_var_) || s.options.bias_acce_var_ < 0)
                return false;
            const Mat3T r = n.R_.matrix();
            if ((r.transpose()*r - Mat3T::Identity()).norm() > 1e-5 ||
                std::abs(r.determinant()-1) > 1e-5) return false;
            for (const Mat18T* m : {&s.covariance, &s.process_noise}) {
                const S tol = S(100)*std::numeric_limits<S>::epsilon()*std::max(S(1), m->norm());
                if ((*m-m->transpose()).norm() > tol) return false;
                Eigen::SelfAdjointEigenSolver<Mat18T> es((*m+m->transpose())*S(0.5));
                if (es.info() != Eigen::Success || es.eigenvalues().minCoeff() < -tol) return false;
            }
            return true;
        }

        bool RestoreSnapshot(const Snapshot& s) {
            if (!ValidSnapshot(s)) return false; // validate before any mutation
            current_time_ = s.nav.timestamp_;
            p_ = s.nav.p_; v_ = s.nav.v_; R_ = s.nav.R_;
            bg_ = s.nav.bg_; ba_ = s.nav.ba_; g_ = s.gravity;
            cov_ = s.covariance; options_ = s.options; Q_ = s.process_noise;
            dx_.setZero();
            return true;
        }

        /**
         * 初始零偏取零
         */
        ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

        /**
         * 设置初始条件
         * @param options 噪声项配置
         * @param init_bg 初始零偏 陀螺
         * @param init_ba 初始零偏 加计
         * @param gravity 重力
         */
        void SetInitialConditions(Options options, const VecT &init_bg, const VecT &init_ba,
                                  const VecT &gravity = VecT(0, 0, -9.8))
        {
            BuildNoise(options);
            options_ = options;
            bg_ = init_bg;
            ba_ = init_ba;
            g_ = gravity;
            cov_ = Mat18T::Identity() * 1e-4;
        }

        /// 使用IMU递推
        bool Predict(const IMU &imu);

        // R order: world position (m), prior right SO(3) innovation (rad).
        // Entries are variances/cross covariances, not standard deviations.
        bool ObserveSE3(const SE3 &pose, const Mat6d& observation_covariance);

        /// accessors
        /// 获取全量状态
        NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

        /// 获取SE3 状态
        SE3 GetNominalSE3() const { return SE3(R_, p_); }

        /// 获取速度
        VecT GetNominalVel() const { return v_; }

        void SetInitialTime(const double t) { current_time_ = t; }

    private:
        void BuildNoise(const Options &options)
        {
            double ev = options.acce_var_;
            double et = options.gyro_var_;
            double eg = options.bias_gyro_var_;
            double ea = options.bias_acce_var_;

            double ev2 = ev; // * ev;
            double et2 = et; // * et;
            double eg2 = eg; // * eg;
            double ea2 = ea; // * ea;

            // 设置过程噪声
            Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

        }

        /// 更新名义状态变量，重置error state
        void UpdateAndReset()
        {
            p_ += dx_.template block<3, 1>(0, 0);
            // std::cout << "before:" << v_.transpose() << std::endl;
            v_ += dx_.template block<3, 1>(3, 0);
            R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

            if (options_.update_bias_gyro_)
            {
                bg_ += dx_.template block<3, 1>(9, 0);
            }

            if (options_.update_bias_acce_)
            {
                ba_ += dx_.template block<3, 1>(12, 0);
            }
            // std::cout << "after v:" << v_.transpose() << ",ba: " << ba_.transpose() << std::endl;
            g_ += dx_.template block<3, 1>(15, 0);

            ProjectCov();
            dx_.setZero();
        }

        /// 对P阵进行投影，参考式(3.59)
        void ProjectCov()
        {
            Mat18T J = Mat18T::Identity();
            J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
            cov_ = J * cov_ * J.transpose();
        }

        /// 成员变量
        double current_time_ = 0.0; // 当前时间

        /// 名义状态
        VecT p_ = VecT::Zero();
        VecT v_ = VecT::Zero();
        SO3 R_;
        VecT bg_ = VecT::Zero();
        VecT ba_ = VecT::Zero();
        VecT g_{0, 0, -9.8};

        /// 误差状态
        Vec18T dx_ = Vec18T::Zero();

        /// 协方差阵
        Mat18T cov_ = Mat18T::Identity();

        /// 噪声阵
        MotionNoiseT Q_ = MotionNoiseT::Zero();

        /// 配置项
        Options options_;
    };

    using ESKFD = ESKF<double>;
    using ESKFF = ESKF<float>;

    template <typename S>
    bool ESKF<S>::Predict(const IMU &imu)
    {
        const double dt = imu.timestamp_ - current_time_;
        if (!imu.quality_.Usable() || !std::isfinite(dt) || !imu.gyro_.allFinite() || !imu.acce_.allFinite() ||
            !(options_.imu_dt_ > 0) || dt <= 0 || dt > 5*options_.imu_dt_) return false;
        const Mat3T old_r = R_.matrix();
        const VecT a = old_r * (imu.acce_ - ba_) + g_;
        const VecT new_p = p_ + v_*dt + S(0.5)*a*dt*dt;
        const VecT new_v = v_ + a*dt;
        const SO3 new_r = R_*SO3::exp((imu.gyro_-bg_)*dt);
        Mat18T F = Mat18T::Identity();
        F.template block<3,3>(0,3) = Mat3T::Identity()*dt;
        F.template block<3,3>(0,6) = -S(0.5)*old_r*SO3::hat(imu.acce_-ba_)*dt*dt;
        F.template block<3,3>(0,12) = -S(0.5)*old_r*dt*dt;
        F.template block<3,3>(0,15) = S(0.5)*Mat3T::Identity()*dt*dt;
        F.template block<3,3>(3,6) = -old_r*SO3::hat(imu.acce_-ba_)*dt;
        F.template block<3,3>(3,12) = -old_r*dt;
        F.template block<3,3>(3,15) = Mat3T::Identity()*dt;
        F.template block<3,3>(6,6) = SO3::exp(-(imu.gyro_-bg_)*dt).matrix();
        F.template block<3,3>(6,9) = -Mat3T::Identity()*dt;
        // Compatibility approximation: retain reference-period discrete Q,
        // scale short/irregular intervals by duration; not a continuous Qc model.
        Mat18T noise_scale=Mat18T::Identity();
        noise_scale.template block<3,3>(3,3)=imu.quality_.accel_noise_scale.cwiseSqrt().template cast<S>().asDiagonal();
        noise_scale.template block<3,3>(6,6)=imu.quality_.gyro_noise_scale.cwiseSqrt().template cast<S>().asDiagonal();
        Mat18T new_cov = F*cov_*F.transpose() + noise_scale*Q_*noise_scale*S(dt/options_.imu_dt_);
        new_cov = ((new_cov+new_cov.transpose())*S(0.5)).eval();
        if (!new_p.allFinite() || !new_v.allFinite() || !new_r.matrix().allFinite() ||
            !new_cov.allFinite()) return false;
        p_=new_p; v_=new_v; R_=new_r; cov_=new_cov;
        dx_=F*dx_; current_time_=imu.timestamp_;
        return true;
    }

    template <typename S>
    bool ESKF<S>::ObserveSE3(const SE3 &pose, const Mat6d& observation_covariance)
    {
        Mat6d validated;
        if (!pose.matrix().allFinite() || !ValidateObservationCovariance(observation_covariance,validated)) return false;
        Eigen::Matrix<S,6,18> H = Eigen::Matrix<S,6,18>::Zero();
        H.template block<3,3>(0,0)=Mat3T::Identity();
        H.template block<3,3>(3,6)=Mat3T::Identity();
        const Eigen::Matrix<S,6,6> V = validated.template cast<S>();
        const Eigen::Matrix<S,6,6> innovation_cov=H*cov_*H.transpose()+V;
        Eigen::LDLT<Eigen::Matrix<S,6,6>> ldlt(innovation_cov);
        if (ldlt.info()!=Eigen::Success || !ldlt.isPositive()) return false;
        const Eigen::Matrix<S,18,6> K=ldlt.solve((cov_*H.transpose()).transpose()).transpose();
        Eigen::Matrix<S,6,1> innovation;
        innovation.template head<3>()=pose.translation().template cast<S>()-p_;
        innovation.template tail<3>()=(R_.inverse()*SO3(pose.unit_quaternion().template cast<S>())).log();
        ESKF candidate=*this;
        candidate.dx_=K*innovation;
        const Mat18T A=Mat18T::Identity()-K*H;
        candidate.cov_=A*cov_*A.transpose()+K*V*K.transpose();
        candidate.UpdateAndReset();
        candidate.cov_=((candidate.cov_+candidate.cov_.transpose())*S(0.5)).eval();
        if (!ValidSnapshot(candidate.ExportSnapshot())) return false;
        *this=candidate;
        return true;
    }

} // namespace zjloc
#endif
