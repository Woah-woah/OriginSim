#include "algo/lidar_geometry_quality.h"
#include "algo/match_quality.h"
#include "algo/lidar_sampling.h"
#include <iostream>
#include <stdexcept>
using namespace zjloc;
void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
int main(){try {
    ESKFD f;auto prior=f.ExportSnapshot();
    prior.nav.R_=SO3::exp(Vec3d(.2,-.5,.3));prior.nav.p_=Vec3d(.3,.1,-.2);
    prior.covariance*=.03;require(f.RestoreSnapshot(prior),"prior");
    const SE3 candidate(prior.nav.R_*SO3::exp(Vec3d(.1,-.2,.3)),prior.nav.p_+Vec3d(.2,.3,-.1));
    const Vec6d innovation=PoseInnovation(prior.nav.GetSE3(),candidate);
    Eigen::Matrix<double,6,18> H=Eigen::Matrix<double,6,18>::Zero();H.block<3,3>(0,0).setIdentity();H.block<3,3>(3,6).setIdentity();
    const Mat6d fixed=Mat6d::Identity()*.01;
    // Independent closed-form legacy diagonal update: P=.03 I, R=.01 I => K=.75 H^T.
    require(f.ObserveSE3(candidate,fixed),"fixed update");auto actual=f.ExportSnapshot();
    require((actual.nav.p_-(prior.nav.p_+.75*innovation.head<3>())).norm()<1e-12,"legacy fixed p");
    require((actual.nav.R_.inverse()*prior.nav.R_*SO3::exp(.75*innovation.tail<3>())).log().norm()<1e-12,"legacy fixed R");
    auto expectedP=prior.covariance;
    for(int i:{0,1,2,6,7,8})expectedP(i,i)=.0075;
    Eigen::Matrix<double,18,18> reset=Eigen::Matrix<double,18,18>::Identity();
    reset.block<3,3>(6,6)=Mat3d::Identity()-.5*SO3::hat(.75*innovation.tail<3>());
    expectedP=(reset*expectedP*reset.transpose()).eval();
    require((actual.covariance-expectedP).norm()<1e-12,"legacy Joseph/reset covariance");
    Mat6d full=fixed;full(0,4)=full(4,0)=.004;
    f.RestoreSnapshot(prior);require(f.ObserveSE3(candidate,full),"cross update");actual=f.ExportSnapshot();
    const Mat6d S=H*prior.covariance*H.transpose()+full;
    const Eigen::Matrix<double,18,6> gain=S.ldlt().solve((prior.covariance*H.transpose()).transpose()).transpose();
    const Vec18d dx=gain*innovation;
    require((actual.nav.p_-prior.nav.p_-dx.head<3>()).norm()<1e-12,"cross terms lost");
    require(std::abs(actual.covariance(0,7))>1e-5,"cross posterior lost");
    MatchQuality quality{true,100,5,4};double score;
    require(AcceptMatch(quality,50,prior,candidate,full,100,score),"same R gate");
    require(std::abs(score-innovation.dot(S.ldlt().solve(innovation)))<1e-12,"gate covariance differs");
    const auto before=f.ExportSnapshot();
    for(int kind=0;kind<5;++kind){Mat6d bad=full;
        if(kind==0)bad(0,0)=-1;
        if(kind==1)bad(0,1)=.1;
        if(kind==2)bad(0,0)=NAN;
        if(kind==3)bad.setZero();
        if(kind==4)bad(0,0)=INFINITY;
        require(!f.ObserveSE3(candidate,bad),"bad R accepted");
        require((f.ExportSnapshot().covariance-before.covariance).norm()==0,"bad R changed P");
        require((f.GetNominalState().p_-before.nav.p_).norm()==0,"bad R changed pose");
        require(!AcceptMatch(quality,50,prior,candidate,bad,100,score),"gate bad R accepted");
    }
    Mat6d huge=Mat6d::Identity()*1e6;
    require(!AcceptMatch(quality,50,prior,SE3(SO3(),Vec3d(20,0,0)),huge,100,score),"R bypass hard gate");
    std::cout<<"PASS fixed legacy equivalence, full SPD cross terms, same-R NIS, atomic invalid R and hard gate\n";
    double max_transport_error=0;
    for(const Vec3d& phi:{Vec3d(1e-8,2e-8,0),Vec3d(.8,-.7,.9),Vec3d(2.,.3,-.4)}){
        Mat3d numerical;const double h=1e-6;
        for(int i=0;i<3;++i){Vec3d d=Vec3d::Zero();d[i]=h;
            numerical.col(i)=((SO3::exp(phi)*SO3::exp(d)).log()-(SO3::exp(phi)*SO3::exp(-d)).log())/(2*h);}
        max_transport_error=std::max(max_transport_error,(numerical-InnovationRotationJacobian(phi)).norm());
        require(max_transport_error<1e-8,"candidate to prior tangent mapping");
    }
    std::cout<<"PASS non-small-angle covariance transport finite difference error="<<max_transport_error<<'\n';
    for(int scene=1;scene<=3;++scene){
        std::vector<LidarCorrespondence> c;
        for(int axis=0;axis<scene;++axis)for(int a=-3;a<=3;++a)for(int b=-3;b<=3;++b){
            Vec3d q;q[axis]=2.;q[(axis+1)%3]=a*.4;q[(axis+2)%3]=b*.5;
            c.push_back({q,q,Vec3d::Unit(axis),1.});}
        auto g=EvaluateLidarGeometry(c,Mat3d::Identity(),Vec3d::Zero(),1.);
        const int expected=scene==1?3:(scene==2?5:6);
        require(g.valid && g.rank==expected,"scene rank wrong");
        require((g.normalized_information-g.directions*g.eigenvalues.asDiagonal()*g.directions.transpose()).norm()<1e-9,"spectrum reconstruction");
        if(scene<=2)require(g.information.col(2).norm()<1e-10,"corridor axial translation should be null");
        auto adaptive=BuildAdaptiveCovariance(g,GeometryOptions{});Mat6d checked;
        require(ValidateObservationCovariance(adaptive,checked),"adaptive not SPD");
        if(scene<3)require((g.directions.col(0).transpose()*adaptive*g.directions.col(0))[0]>99.,"null direction variance shrunk");
        std::cout<<"PASS geometry scene="<<scene<<" rank="<<g.rank<<" spectrum="<<g.eigenvalues.transpose()<<'\n';
    }
    // Normal directions alone are insufficient: points all at origin have zero rotational information.
    std::vector<LidarCorrespondence> centered;
    for(int i=0;i<3;++i)centered.push_back({Vec3d::Zero(),Vec3d::Zero(),Vec3d::Unit(i),1.});
    require(EvaluateLidarGeometry(centered,Mat3d::Identity(),Vec3d::Zero(),1.).rank==3,"false full-rank normals");
    std::vector<LidarCorrespondence> one={{Vec3d(1,2,3),Vec3d::Zero(),Vec3d::UnitX(),.2}};
    auto g=EvaluateLidarGeometry(one,Mat3d::Identity(),Vec3d::Zero(),2.);
    auto J=PointPlaneJacobian(one[0].end_imu,one[0].normal_world,Mat3d::Identity());
    require((g.information-.04*J.transpose()*J).norm()<1e-12,"weight must square");
    one[0].end_imu.x()=10;g=EvaluateLidarGeometry(one,Mat3d::Identity(),Vec3d::Zero(),2.);
    J=PointPlaneJacobian(one[0].end_imu,one[0].normal_world,Mat3d::Identity());
    require((g.information-.01*J.transpose()*J).norm()<1e-12,"Huber IRLS scaling");
    std::vector<LidarCorrespondence> pool;
    for(int axis=0;axis<3;++axis)for(int i=0;i<(axis==0?120:30);++i){
        Vec3d q;q[axis]=2;q[(axis+1)%3]=(i%10-5)*.2;q[(axis+2)%3]=(i/10-3)*.3;
        pool.push_back({q,q,Vec3d::Unit(axis),1.});}
    SamplingCoverage coverage,again;const auto selected=SelectGeometricCoverage(pool,60,coverage);
    require(selected==SelectGeometricCoverage(pool,60,again),"sampling nondeterminism");
    std::vector<LidarCorrespondence> retained;for(size_t i:selected)retained.push_back(pool[i]);
    const auto balanced=EvaluateLidarGeometry(retained,Mat3d::Identity(),Vec3d::Zero(),1.);
    std::vector<LidarCorrespondence> first(pool.begin(),pool.begin()+60);
    const auto prefix=EvaluateLidarGeometry(first,Mat3d::Identity(),Vec3d::Zero(),1.);
    require(selected.size()==60 && balanced.rank==6 && prefix.rank==3,"same-budget coverage improvement");
    auto opposite=pool[0];opposite.normal_world*=-1;
    require(CorrespondenceBin(opposite)==CorrespondenceBin(pool[0]),"normal sign duplicated bucket");
    std::cout<<"PASS deterministic same-budget stratification ranks="<<prefix.rank<<"->"<<balanced.rank
             <<" normals="<<coverage.normal_bins<<" spatial="<<coverage.spatial_bins<<'\n';
    ESKFD strong,weak;strong.RestoreSnapshot(prior);weak.RestoreSnapshot(prior);Mat6d weakR=fixed;weakR(0,0)=100;
    strong.ObserveSE3(candidate,fixed);weak.ObserveSE3(candidate,weakR);
    require(std::abs(weak.GetNominalState().p_.x()-prior.nav.p_.x())<std::abs(strong.GetNominalState().p_.x()-prior.nav.p_.x()),"weak direction not suppressed");
    std::cout<<"PASS false-full-rank guard, squared residual weight, Huber IRLS, weak-direction attenuation\n";
} catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;} }
