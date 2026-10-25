#pragma once
#include "algo/eskf.hpp"
namespace zjloc {
struct MatchQuality {
    bool usable=false;
    int residuals=0;
    double initial_cost=0, final_cost=0;
    const char* reason="solver_unusable";
};
inline bool AcceptMatch(const MatchQuality& q, int minimum_support,
                        const ESKFD::Snapshot& prior, const SE3& candidate,
                        const Mat6d& observation_covariance, double max_score, double& score,
                        const char** reason=nullptr, double max_translation=2., double max_rotation=1.) {
    auto reject=[&](const char* why){if(reason)*reason=why;return false;};
    score=std::numeric_limits<double>::infinity();
    if (!q.usable) return reject("solver_unusable");
    if (q.residuals<minimum_support) return reject("insufficient_support");
    if (!candidate.matrix().allFinite() || !ESKFD::ValidSnapshot(prior)) return reject("invalid_state");
    if (!std::isfinite(q.initial_cost) || !std::isfinite(q.final_cost) || q.final_cost<0 ||
        q.final_cost>q.initial_cost+1e-8*std::max(1.0,q.initial_cost)) return reject("invalid_cost");
    const Vec6d innovation=PoseInnovation(prior.nav.GetSE3(),candidate);
    if (!innovation.allFinite() || innovation.head<3>().norm()>max_translation ||
        innovation.tail<3>().norm()>max_rotation) return reject("hard_innovation_limit");
    Mat6d R;
    if (!ValidateObservationCovariance(observation_covariance,R)) return reject("invalid_R");
    Eigen::Matrix<double,6,18> H=Eigen::Matrix<double,6,18>::Zero();
    H.block<3,3>(0,0)=Mat3d::Identity();H.block<3,3>(3,6)=Mat3d::Identity();
    const Mat6d S=H*prior.covariance*H.transpose()+R;
    Eigen::LDLT<Mat6d> ldlt(S);
    if(ldlt.info()!=Eigen::Success || !ldlt.isPositive()) return reject("invalid_S");
    score=innovation.dot(ldlt.solve(innovation));
    if (!std::isfinite(score) || score<0 || score>max_score) return reject("innovation_score");
    if(reason)*reason="accepted";
    return true;
}
} // namespace zjloc
