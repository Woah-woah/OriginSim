#pragma once
#include "algo/eskf.hpp"
namespace zjloc {
struct OdomProjection {
    SE3 pose;
    Vec3d linear, angular;
    Mat6d pose_cov, twist_cov;
};
// T_IB maps child B vectors to IMU I; translation is the lever arm in I.
// Pose orientation covariance uses fixed world axes (right error -> world R*dtheta).
inline OdomProjection ProjectOdometry(const ESKFD::Snapshot& s, const Vec3d& omega,
                                     const Vec3d& gyro_variance, const SE3& T_IB=SE3()) {
    OdomProjection out;
    const Mat3d R=s.nav.R_.matrix(), C=T_IB.rotationMatrix().transpose();
    const Vec3d r=T_IB.translation(), vi=R.transpose()*s.nav.v_;
    out.pose=s.nav.GetSE3()*T_IB;
    out.linear=C*(vi+omega.cross(r)); out.angular=C*omega;
    Eigen::Matrix<double,6,18> J=Eigen::Matrix<double,6,18>::Zero();
    J.block<3,3>(0,0)=Mat3d::Identity();
    J.block<3,3>(0,6)=-R*SO3::hat(r);
    J.block<3,3>(3,6)=R;
    out.pose_cov=J*s.covariance*J.transpose();
    J.setZero();
    J.block<3,3>(0,3)=C*R.transpose();
    J.block<3,3>(0,6)=C*SO3::hat(vi);
    J.block<3,3>(0,9)=C*SO3::hat(r);
    J.block<3,3>(3,9)=-C;
    Eigen::Matrix<double,6,3> G;
    G.topRows<3>()=-C*SO3::hat(r); G.bottomRows<3>()=C;
    // First-order approximation: instantaneous measurement noise independent of P.
    out.twist_cov=J*s.covariance*J.transpose()+G*gyro_variance.asDiagonal()*G.transpose();
    return out;
}
} // namespace zjloc
