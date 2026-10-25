#include "algo/imu_odom_predictor.h"
#include "algo/odom_projection.h"
#include "algo/match_quality.h"
#include "algo/aux_time_crop.h"
#include "algo/frame_commit.h"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace zjloc;
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void near(double a,double b,double eps=1e-9) {check(std::abs(a-b)<eps,"numeric mismatch");}
LioStateUpdate initial(double t=1.0) {
    ESKFD f; auto s=f.ExportSnapshot(); s.nav.timestamp_=t; s.gravity=Vec3d(0,0,-9.81);
    return {s,1,t,Vec3d::Constant(1e-5)};
}
IMU sample(double t,Vec3d acc=Vec3d(0,0,9.81),Vec3d gyro=Vec3d::Zero()) { return IMU(t,gyro,acc); }
int main() {
try {
    ESKFD f; auto s=initial().state;
    s.nav.bg_=Vec3d(.1,.2,.3); s.nav.ba_=Vec3d(.2,.1,.3);
    check(f.RestoreSnapshot(s),"restore"); auto x=f.ExportSnapshot();
    near((x.nav.bg_-s.nav.bg_).norm(),0); near((x.covariance-s.covariance).norm(),0);
    auto bad=s; bad.covariance(0,0)=-1; check(!f.RestoreSnapshot(bad),"invalid P accepted");
    near((f.ExportSnapshot().nav.bg_-s.nav.bg_).norm(),0);
    check(!f.Predict(sample(1)),"duplicate"); check(!f.Predict(sample(.99)),"reverse");
    check(!f.Predict(sample(1.2)),"gap"); near(f.GetNominalState().timestamp_,1);
    near((f.ExportSnapshot().covariance-s.covariance).norm(),0);
    check(!f.ObserveSE3(SE3(),-Mat6d::Identity()),"negative variance");
    check(f.ObserveSE3(SE3(),Mat6d::Identity()*.01),"matrix correction");
    check(f.ObserveSE3(SE3(),Mat6d::Identity()*.01),"repeat correction");
    std::cout<<"PASS snapshot, atomic rejection, zero/reverse/gap dt, matrix Joseph update\n";
    for(int mode=0;mode<4;++mode) {
        ImuOdomPredictor p(3,3); auto u=initial();
        if(mode==1) u.state.nav.v_=Vec3d(1,0,0);
        p.Submit(u);
        for(int i=0;i<=400;++i) {
            p.Append(sample(1+.005*i,Vec3d(mode==2?2:0,0,9.81),mode==3?Vec3d(.3,-.4,.6):Vec3d::Zero())); p.Advance();
        }
        check(p.Publishable(),"not publishable"); auto n=p.Snapshot().nav;
        if(mode==0) near(n.p_.norm(),0);
        if(mode==1) near(n.p_.x(),2);
        if(mode==2) {near(n.p_.x(),4);near(n.v_.x(),4);}
        if(mode==3) near((n.R_.inverse()*SO3::exp(Vec3d(.6,-.8,1.2))).log().norm(),0);
    }
    std::cout<<"PASS stationary 2s, constant velocity, acceleration, compound rotation\n";
    ImuOdomPredictor early(2,2),late(2,2); auto u=initial(1.002); u.state.nav.v_=Vec3d(.7,0,0);
    early.Submit(u);
    for(int i=0;i<=40;++i) {
        auto imu=sample(1+.005*i,Vec3d(.2*i,0,9.81),Vec3d(.3,-.2,.1));
        early.Append(imu); late.Append(imu);
        if(i==17)late.Submit(u);
        early.Advance();late.Advance();
    }
    near((early.Snapshot().nav.p_-late.Snapshot().nav.p_).norm(),0);
    near((early.Snapshot().covariance-late.Snapshot().covariance).norm(),0,1e-12);
    check(!late.Submit(u),"duplicate state"); u.sequence=0;check(!late.Submit(u),"zero sequence");
    check(!late.Append(sample(1.1)),"stale imu");
    std::cout<<"PASS off-grid posterior, 83ms late initialization replay, P equality, ordering\n";
    // A correction at the already-published tail is installed internally; ROS worker de-duplicates outputs.
    u=initial(1.2);u.sequence=2;u.state.nav.p_=Vec3d(2,0,0);
    check(late.Submit(u),"new correction");late.Advance();near(late.Snapshot().nav.p_.x(),2);
    u=initial(1.205);u.sequence=1;check(!late.Submit(u),"stale sequence");
    u=initial(1.21);u.sequence=3;late.Submit(u);late.Advance();near(late.Snapshot().nav.timestamp_,1.2);
    late.Append(sample(1.215));late.Advance();near(late.Snapshot().nav.timestamp_,1.215);
    late.Append(sample(1.5));late.Advance();check(late.Faulted(),"gap must fault");
    ImuOdomPredictor timeout(2,.1);timeout.Submit(initial());
    for(int i=0;i<50;++i){timeout.Append(sample(1+i*.005));timeout.Advance();}
    check(!timeout.Publishable(),"prediction timeout");
    u=initial(1.245);u.sequence=2;u.last_accepted_lidar_stamp=1;
    timeout.Submit(u);timeout.Advance();check(!timeout.Publishable(),"rejected match refreshed timeout");
    u.sequence=3;u.state.nav.timestamp_=1.25;u.last_accepted_lidar_stamp=1.25;
    timeout.Append(sample(1.25));timeout.Submit(u);timeout.Advance();check(timeout.Publishable(),"correction recovery");
    std::cout<<"PASS pending future, gap fail-closed, timeout, rejected-frame age, recovery\n";
    // Finite-difference all 18 columns of pose/twist projections with lever arm and nonzero rotations.
    s=initial().state;s.nav.R_=SO3::exp(Vec3d(.5,-.7,.9));s.nav.v_=Vec3d(1,2,3);
    const Vec3d w(.8,-.3,.4);const SE3 T(SO3::exp(Vec3d(-.4,.2,.9)),Vec3d(.2,-.3,.5));
    auto nominal=ProjectOdometry(s,w,Vec3d::Zero(),T);
    Eigen::Matrix<double,6,18> jp,jt;const double h=1e-6;
    for(int k=0;k<18;++k){
        auto a=s,b=s;Vec3d wa=w,wb=w;Vec3d delta=Vec3d::Zero();delta[k%3]=h;
        if(k<3){a.nav.p_+=delta;b.nav.p_-=delta;}
        else if(k<6){a.nav.v_+=delta;b.nav.v_-=delta;}
        else if(k<9){a.nav.R_=s.nav.R_*SO3::exp(delta);b.nav.R_=s.nav.R_*SO3::exp(-delta);}
        else if(k<12){wa-=delta;wb+=delta;}
        auto pa=ProjectOdometry(a,wa,Vec3d::Zero(),T),pb=ProjectOdometry(b,wb,Vec3d::Zero(),T);
        jp.col(k).head<3>()=(pa.pose.translation()-pb.pose.translation())/(2*h);
        jp.col(k).tail<3>()=(pa.pose.so3()*pb.pose.so3().inverse()).log()/(2*h);
        jt.col(k).head<3>()=(pa.linear-pb.linear)/(2*h);jt.col(k).tail<3>()=(pa.angular-pb.angular)/(2*h);
    }
    near((jp*s.covariance*jp.transpose()-nominal.pose_cov).norm(),0,1e-8);
    near((jt*s.covariance*jt.transpose()-nominal.twist_cov).norm(),0,1e-8);
    near((nominal.linear-T.rotationMatrix().transpose()*(s.nav.R_.inverse()*s.nav.v_+w.cross(T.translation()))).norm(),0);
    std::cout<<"PASS lever arm, frame rotations, finite-difference pose/twist covariance\n";
    ImuOdomPredictor history(.1,1);for(int i=0;i<100;++i)history.Append(sample(1+i*.005));
    history.Submit(initial());history.Advance();check(history.counters.history_miss==1,"history rejection");
    check(history.HistorySize()<=23,"unbounded history");
    auto nan=sample(2);nan.acce_[0]=std::numeric_limits<double>::quiet_NaN();check(!history.Append(nan),"NaN");
    std::cout<<"PASS bounded history, expired posterior, NaN input\n";
    MatchQuality quality{true,100,10,5};s=initial().state;double score=0;
    check(AcceptMatch(quality,50,s,SE3(SO3(),Vec3d(.1,0,0)),Mat6d::Identity()*.01,100,score),"normal match");
    check(!AcceptMatch(quality,50,s,SE3(SO3(),Vec3d(50,0,0)),Mat6d::Identity()*.01,100,score),"outlier match");
    s.nav.p_=Vec3d(50,0,0);s.nav.v_=Vec3d(20,0,0);
    check(AcceptMatch(quality,50,s,SE3(SO3(),Vec3d(50.1,0,0)),Mat6d::Identity()*.01,100,score),"fast true motion");
    quality.residuals=10;check(!AcceptMatch(quality,50,s,s.nav.GetSE3(),Mat6d::Identity()*.01,100,score),"weak support");
    quality.residuals=100;quality.final_cost=20;check(!AcceptMatch(quality,50,s,s.nav.GetSE3(),Mat6d::Identity()*.01,100,score),"diverging solve");
    struct Point{double timestamp;Vec3d raw_point=Vec3d::Zero();};
    std::vector<Point> points={{1.1},{1.3},{1.0},{1.2},{.9},{std::numeric_limits<double>::quiet_NaN()}};
    SortFinitePoints(points);check(points.size()==5,"finite points");size_t cursor=0;
    std::vector<Point> first,second;CropAuxPoints(points,cursor,1.0,1.2,0,first);
    check(first.size()==2 && points[cursor].timestamp==1.2,"half open boundary");
    CropAuxPoints(points,cursor,1.2,1.4,0,second);check(second.size()==2 && cursor==5,"future retention");
    cursor=0;first.clear();CropAuxPoints(points,cursor,1.1,1.3,.1,first);check(first.size()==2,"offset crop");
    std::cout<<"PASS match support/cost/innovation gates, true fast motion, finite sorted aux, adjacent windows/offset\n";
    ESKFD inner,outer;auto anchor=initial().state;inner.RestoreSnapshot(anchor);
    auto left=sample(1,Vec3d(.2,0,9.81)),right=sample(1.005,Vec3d(.8,0,9.81));
    check(PropagateInterval(inner,left,right,1.002),"inner partial interval");
    outer.RestoreSnapshot(inner.ExportSnapshot());
    check(PropagateInterval(inner,left,right,1.005),"inner remainder");
    check(PropagateInterval(outer,left,right,1.005),"outer remainder");
    near((inner.ExportSnapshot().covariance-outer.ExportSnapshot().covariance).norm(),0,1e-12);
    near((inner.GetNominalState().p_-outer.GetNominalState().p_).norm(),0,1e-12);
    std::cout<<"PASS shared LiDAR-end interpolation and truncated IMU interval\n";
    // Neither a yaw branch crossing nor a near-vertical attitude uses Euler subtraction.
    for(const Vec3d& angle : {Vec3d(0,0,3.14),Vec3d(0,1.5707,0)}){
        ESKFD turning;auto start=initial().state;start.nav.R_=SO3::exp(angle);turning.RestoreSnapshot(start);
        check(turning.Predict(sample(1.005,Vec3d(0,0,9.81),Vec3d(.2,.3,2))),"turning prediction");
        near((turning.GetNominalState().R_.inverse()*start.nav.R_*SO3::exp(Vec3d(.2,.3,2)*.005)).log().norm(),0);
        check(ESKFD::ValidSnapshot(turning.ExportSnapshot()),"turning covariance");
    }
    // Already-running filter: late posterior replaces a deliberately different old trajectory.
    ImuOdomPredictor ontime(2,2),delayed(2,2);auto old=initial();
    old.state.nav.v_=Vec3d(1,0,0);ontime.Submit(old);delayed.Submit(old);
    auto corrected=initial(1.102);corrected.sequence=2;corrected.state.nav.p_=Vec3d(.08,0,0);
    corrected.state.nav.v_=Vec3d(.9,0,0);corrected.state.nav.bg_=Vec3d(.01,.02,.03);
    for(int i=0;i<=60;++i){
        auto imu=sample(1+.005*i,Vec3d(.3,0,9.81),Vec3d(.1,.2,.3));
        ontime.Append(imu);delayed.Append(imu);
        if(i==21)ontime.Submit(corrected);
        if(i==37)delayed.Submit(corrected);
        ontime.Advance();delayed.Advance();
    }
    near((ontime.Snapshot().nav.p_-delayed.Snapshot().nav.p_).norm(),0,1e-12);
    near((ontime.Snapshot().nav.R_.inverse()*delayed.Snapshot().nav.R_).log().norm(),0,1e-12);
    near((ontime.Snapshot().covariance-delayed.Snapshot().covariance).norm(),0,1e-12);
    struct DerivedState {Eigen::Quaterniond rotation;Vec3d translation,velocity,ba,bg;};
    struct RegisteredPoint {Vec3d end_point,point;};
    DerivedState cache;auto committed=corrected.state.nav;
    CommitFrameState(committed,cache);
    near((cache.translation-committed.p_).norm(),0);near((cache.bg-committed.bg_).norm(),0);
    std::vector<RegisteredPoint> products={{Vec3d(1,2,3),Vec3d(99,99,99)}};
    RegisterCommittedPoints(committed.GetSE3(),products);
    near((products[0].point-committed.GetSE3()*products[0].end_point).norm(),0);
    std::cout<<"PASS posterior state cache and registered point refresh\n";
    std::cout<<"PASS yaw branch, near-vertical attitude, late correction of running trajectory\nALL CORE TESTS PASSED\n";
} catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
