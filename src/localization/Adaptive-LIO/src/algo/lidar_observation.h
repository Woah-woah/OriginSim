#pragma once
#include "common/eigen_types.h"
#include <limits>
namespace zjloc {
inline bool ValidateObservationCovariance(const Mat6d& input,Mat6d& output) {
    if (!input.allFinite()) return false;
    const double tolerance=100*std::numeric_limits<double>::epsilon()*std::max(1.,input.norm());
    if((input-input.transpose()).norm()>tolerance) return false;
    const Mat6d symmetric=(input+input.transpose())*.5;
    Eigen::LLT<Mat6d> llt(symmetric);
    if(llt.info()!=Eigen::Success) return false;
    output=symmetric;
    return true;
}
inline Vec6d PoseInnovation(const SE3& prior,const SE3& candidate) {
    Vec6d v;v.head<3>()=candidate.translation()-prior.translation();
    v.tail<3>()=(prior.so3().inverse()*candidate.so3()).log();return v;
}
// d Log(Exp(phi) Exp(delta)) / d delta: inverse SO(3) right Jacobian.
inline Mat3d InnovationRotationJacobian(const Vec3d& phi) {
    const double theta=phi.norm();const Mat3d h=SO3::hat(phi);
    const double a=theta<1e-5 ? 1./12.+theta*theta/720. :
        (1.-.5*theta/std::tan(.5*theta))/(theta*theta);
    return Mat3d::Identity()+.5*h+a*h*h;
}
inline Mat6d TransportCandidateCovariance(const SE3& prior,const SE3& candidate,const Mat6d& covariance) {
    Mat6d J=Mat6d::Identity();
    J.bottomRightCorner<3,3>()=InnovationRotationJacobian(PoseInnovation(prior,candidate).tail<3>());
    return J*covariance*J.transpose();
}
} // namespace zjloc
