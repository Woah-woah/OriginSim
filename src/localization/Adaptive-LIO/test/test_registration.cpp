#include "algo/point_time.h"
#include "lio/lidarFactor.h"
#include <ceres/manifold.h>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace zjloc;
void require(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
int main() { try {
    const SE3 extrinsic(SO3::exp(Vec3d(.3,-.2,.4)),Vec3d(.2,.1,-.15));
    double max_error=0;
    for (int mode=0;mode<4;++mode) {
        const Vec3d omega=mode>=2?Vec3d(.4,-.3,.7):Vec3d::Zero();
        const Vec3d velocity=(mode==1||mode==3)?Vec3d(.8,-.4,.2):Vec3d::Zero();
        auto pose=[&](double t){return SE3(SO3::exp(omega*t),velocity*t);};
        std::vector<NavStated> trajectory;
        for(double t:{0.,.01,.02,.03,.04}) trajectory.emplace_back(t,pose(t).so3(),pose(t).translation());
        const Vec3d world(3,-1,2);
        for(double t:{0.,.004,.01,.023,.04}) {
            const Vec3d raw=extrinsic.inverse()*pose(t).inverse()*world;
            Vec3d end;
            require(DeskewToEndImu(raw,t,trajectory,extrinsic,pose(.04),end),"deskew coverage");
            max_error=std::max(max_error,(pose(.04)*end-world).norm());
            require(max_error<1e-11,"world reconstruction or repeated compensation");
            // Applying deskew from immutable raw again is identical, never double compensates.
            Vec3d repeat;require(DeskewToEndImu(raw,t,trajectory,extrinsic,pose(.04),repeat),"repeat");
            require((end-repeat).norm()<1e-14,"immutable raw");
        }
        Vec3d unchanged(7,8,9);
        require(!DeskewToEndImu(world,-.001,trajectory,extrinsic,pose(.04),unchanged),"before history");
        require(!DeskewToEndImu(world,.041,trajectory,extrinsic,pose(.04),unchanged),"after history");
        require((unchanged-Vec3d(7,8,9)).norm()==0,"failure mutated output");
        // Aux timestamp offset and nontrivial aux->main extrinsic, same physical world point.
        const SE3 aux(SO3::exp(Vec3d(-.2,.5,.1)),Vec3d(-.3,.2,.1));
        const double raw_aux_stamp=.013,offset=.007,t=raw_aux_stamp+offset;
        const Vec3d aux_raw=aux.inverse()*extrinsic.inverse()*pose(t).inverse()*world;
        Vec3d end;require(DeskewToEndImu(aux*aux_raw,t,trajectory,extrinsic,pose(.04),end),"aux deskew");
        require((pose(.04)*end-world).norm()<1e-11,"aux offset/extrinsic");
    }
    std::cout<<"PASS static/translation/rotation/combined deskew; nonidentity extrinsics, endpoints, aux; max_error="<<max_error<<'\n';
    const Vec3d target=extrinsic*Vec3d(1.2,-.4,2),anchor(.4,.2,-.8),normal=Vec3d(.3,-.7,.5).normalized();
    const double weight=.7,h=1e-6;
    Eigen::Quaterniond q=SO3::exp(Vec3d(.5,-.8,.2)).unit_quaternion();Vec3d p(.3,.6,-.2);
    std::unique_ptr<ceres::CostFunction> factor(CT_ICP::PointToPlaneFunctor::Create(anchor,target,normal,weight));
    const double* params[]={p.data(),q.coeffs().data()};double residual;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> jp;Eigen::Matrix<double,1,4,Eigen::RowMajor> jq;
    double* jac[]={jp.data(),jq.data()};require(factor->Evaluate(params,&residual,jac),"factor evaluate");
    Eigen::Matrix<double,1,6> numeric;
    for(int k=0;k<6;++k){
        Vec3d delta=Vec3d::Zero();delta[k%3]=h;Vec3d a=p,b=p;auto qa=q,qb=q;
        if(k<3){a+=delta;b-=delta;}else{qa=q*SO3::exp(delta).unit_quaternion();qb=q*SO3::exp(-delta).unit_quaternion();}
        const double* pa[]={a.data(),qa.coeffs().data()};const double* pb[]={b.data(),qb.coeffs().data()};
        double ra,rb;factor->Evaluate(pa,&ra,nullptr);factor->Evaluate(pb,&rb,nullptr);numeric[k]=(ra-rb)/(2*h);
    }
    Eigen::Matrix<double,1,6> expected;expected.head<3>()=weight*normal.transpose();
    expected.tail<3>()=-weight*normal.transpose()*q.toRotationMatrix()*SO3::hat(target);
    const double error=(numeric-expected).norm();require(error<1e-8,"physical right tangent Jacobian");
    // Ceres Eigen manifold is LEFT half-angle: delta_ceres=R*delta_right/2.
    ceres::EigenQuaternionManifold manifold;
    Eigen::Matrix<double,4,3,Eigen::RowMajor> plus;manifold.PlusJacobian(q.coeffs().data(),plus.data());
    require((jq*plus*q.toRotationMatrix()*.5-expected.tail<3>()).norm()<1e-10,"Ceres tangent convention");
    require((jp-expected.head<3>()).norm()<1e-12,"translation Jacobian");
    std::cout<<"PASS actual Ceres factor right perturbation and EigenQuaternionManifold; error="<<error<<'\n';
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;} }
