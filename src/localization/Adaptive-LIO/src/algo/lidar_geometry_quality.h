#pragma once
#include "algo/lidar_observation.h"
#include <vector>
#include <algorithm>
namespace zjloc {
struct LidarCorrespondence { Vec3d end_imu, anchor_world, normal_world; double weight=1.; };
struct GeometryOptions {
    double reference_length=1.; // metres; z translation is p/reference_length
    double variance_floor=.01,variance_ceiling=100.; // normalized z units squared
    double residual_variance=1.; // m^2, experimental until calibrated
    double correlation_inflation=1.;
    bool Valid() const {
        return std::isfinite(reference_length) && reference_length>0 &&
            std::isfinite(variance_floor) && variance_floor>0 && std::isfinite(variance_ceiling) &&
            variance_ceiling>=variance_floor && std::isfinite(residual_variance) && residual_variance>0 &&
            std::isfinite(correlation_inflation) && correlation_inflation>=1;
    }
};
struct LidarGeometryQuality {
    Mat6d information=Mat6d::Zero(),normalized_information=Mat6d::Zero(),directions=Mat6d::Identity();
    Vec6d eigenvalues=Vec6d::Zero(); // ascending, normalized; directions include weak subspace
    int rank=0; size_t count=0;
    double information_per_point=0,reference_length=1.;
    bool valid=false;
};
inline Eigen::Matrix<double,1,6> PointPlaneJacobian(const Vec3d& q,const Vec3d& n,const Mat3d& R) {
    Eigen::Matrix<double,1,6> J;J.head<3>()=n.transpose();J.tail<3>()=-n.transpose()*R*SO3::hat(q);return J;
}
inline LidarGeometryQuality EvaluateLidarGeometry(const std::vector<LidarCorrespondence>& points,
                                                  const Mat3d& rotation,const Vec3d& position,double length) {
    LidarGeometryQuality result;result.reference_length=length;
    if(points.empty() || !rotation.allFinite() || !position.allFinite() || !std::isfinite(length) || length<=0) return result;
    for(const auto& c:points) {
        if(!c.end_imu.allFinite() || !c.anchor_world.allFinite() || !c.normal_world.allFinite() ||
           std::abs(c.normal_world.norm()-1)>1e-6 || !std::isfinite(c.weight) || c.weight<=0) return result;
        const double weighted_residual=c.weight*c.normal_world.dot(rotation*c.end_imu+position-c.anchor_world);
        // Huber delta=.5 on the ACTUAL weighted Ceres residual. Nonnegative IRLS approximation.
        const double robust=std::abs(weighted_residual)<=.5 ? 1. : .5/std::abs(weighted_residual);
        const auto J=PointPlaneJacobian(c.end_imu,c.normal_world,rotation);
        result.information.noalias()+=c.weight*c.weight*robust*J.transpose()*J;
    }
    Mat6d D=Mat6d::Identity();D.topLeftCorner<3,3>()*=length;
    result.normalized_information=D*result.information*D;
    Eigen::SelfAdjointEigenSolver<Mat6d> es(result.normalized_information);
    if(es.info()!=Eigen::Success || !es.eigenvalues().allFinite()) return result;
    const double tol=std::max(1e-9,es.eigenvalues().maxCoeff()*1e-6);
    if(es.eigenvalues().minCoeff() < -tol) return result;
    result.eigenvalues=es.eigenvalues().cwiseMax(0.); result.directions=es.eigenvectors();
    result.rank=(result.eigenvalues.array()>tol).count();result.count=points.size();
    result.information_per_point=result.eigenvalues.sum()/result.count;result.valid=true;
    return result;
}
inline Mat6d BuildAdaptiveCovariance(const LidarGeometryQuality& g,const GeometryOptions& options) {
    if(!g.valid || !options.Valid() || std::abs(g.reference_length-options.reference_length)>1e-12)
        return Mat6d::Constant(std::numeric_limits<double>::quiet_NaN());
    Vec6d variances;
    for(int i=0;i<6;++i) variances[i]=g.eigenvalues[i]<=1e-12 ? options.variance_ceiling :
        std::clamp(options.residual_variance*options.correlation_inflation/g.eigenvalues[i],
                   options.variance_floor,options.variance_ceiling);
    Mat6d D=Mat6d::Identity();D.topLeftCorner<3,3>()*=options.reference_length;
    return D*g.directions*variances.asDiagonal()*g.directions.transpose()*D;
}
} // namespace zjloc
