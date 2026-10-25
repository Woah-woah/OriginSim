#include "algo/imu_odom_predictor.h"
#include "algo/imu_startup.h"
#include <iostream>
#include <stdexcept>
using namespace zjloc;
void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
IMU sample(double t){return IMU(t,Vec3d(.2,-.1,.3),Vec3d(.1,.2,9.8));}
int main(){try {
    using Decision=ImuQualityGate::Decision;
    ImuStartupGate startup;
    for (double stamp : {1.0, 1.38, 1.50, 1.59, 3.36})
        require(!startup.Ready(sample(stamp)), "irregular startup passed");
    for (int index=1; index<200; ++index)
        require(!startup.Ready(sample(3.36+index*.005)), "startup settled too early");
    auto first_ready=sample(4.365);
    require(startup.Ready(first_ready), "continuous startup never ready");
    ImuQualityGate startup_quality;
    require(startup_quality.Process(first_ready)==Decision::Forward, "startup poisoned quality gate");
    auto runtime_gap=sample(4.5);
    require(startup.Ready(runtime_gap), "startup gate hides runtime gap");
    require(startup_quality.Process(runtime_gap)==Decision::Fault, "runtime gap protection disabled");
    ImuStartupGate invalid_startup;
    require(!invalid_startup.Ready(sample(1)), "startup skipped settling");
    require(!invalid_startup.Ready(sample(NAN)), "nonfinite startup passed");
    require(!invalid_startup.Ready(sample(10)), "nonfinite input failed to reset settling");
    ImuQualityOptions options;options.gyro_limit=Vec3d::Constant(2);options.accel_limit=Vec3d::Constant(20);
    ImuQualityGate no_hold(options);auto s=sample(1.);require(no_hold.Process(s)==Decision::Forward,"healthy");
    s=sample(1.005);s.gyro_.x()=1.95;require(no_hold.Process(s)==Decision::Fault,"default saturation didn't stop");
    require(s.quality_.saturation_mask==1 && !(s.quality_.valid_mask&1),"single-axis mask");
    s=sample(1.01);require(no_hold.Process(s)==Decision::Fault,"fault not latched");
    ImuQualityGate nonfinite(options);s=sample(1);nonfinite.Process(s);s=sample(1.005);s.acce_.y()=NAN;
    require(nonfinite.Process(s)==Decision::Fault,"nonfinite no hold");
    ImuQualityGate timing(options);s=sample(2);timing.Process(s);s=sample(2);require(timing.Process(s)==Decision::Drop,"duplicate");
    s=sample(1.99);require(timing.Process(s)==Decision::Drop,"minor reorder");
    s=sample(.5);require(timing.Process(s)==Decision::Fault,"clock reset");
    ImuQualityGate gap(options);s=sample(1);gap.Process(s);s=sample(1.051);require(gap.Process(s)==Decision::Fault,"gap");
    ImuQualityGate bad_stamp(options);s=sample(NAN);require(bad_stamp.Process(s)==Decision::Drop,"NaN time");
    s=sample(1);require(bad_stamp.Process(s)==Decision::Forward,"NaN time poisoned watermark");
    options.hold_seconds=.025;options.recovery_samples=2;
    ImuQualityGate gate(options);std::vector<IMU> sequence;
    for(int i=0;i<=6;++i){s=sample(1+i*.005);if(i==1||i==2)s.gyro_.x()=2.;
        require(gate.Process(s)==Decision::Forward,"finite hold/recovery");sequence.push_back(s);}
    require(sequence[1].quality_.held_mask==1 && sequence[1].gyro_.x()==.2,"axis substitution");
    require(sequence[3].quality_.degraded && !sequence[4].quality_.degraded,"recovery hysteresis");
    require(sequence[4].quality_.gyro_noise_scale.isOnes(),"healthy base noise not restored");
    ESKFD inner;inner.SetInitialTime(1.);const auto initial=inner.ExportSnapshot();
    require(PropagateInterval(inner,sequence[0],sequence[1],1.005),"hold propagation");
    require(PropagateInterval(inner,sequence[1],sequence[2],1.007),"degraded boundary interpolation");
    const auto anchor=inner.ExportSnapshot();
    for(int i=2;i<=6;++i)require(PropagateInterval(inner,sequence[i-1],sequence[i],sequence[i].timestamp_),"inner remainder");
    ImuOdomPredictor replay(2,2);for(const auto& imu:sequence)replay.Append(imu);
    require(replay.Submit({anchor,1,1.007,Vec3d::Constant(1e-5)}),"late degraded anchor");replay.Advance();
    require(replay.Publishable(),"replay invalid");
    require((inner.GetNominalState().p_-replay.Snapshot().nav.p_).norm()<1e-12,"degraded p replay mismatch");
    require((inner.ExportSnapshot().covariance-replay.Snapshot().covariance).norm()<1e-12,"degraded P replay mismatch");
    require((inner.ExportSnapshot().process_noise-initial.process_noise).norm()==0,"hold poisoned Q");
    ESKFD healthy;healthy.RestoreSnapshot(initial);for(int i=1;i<=6;++i){auto a=sample(sequence[i-1].timestamp_),b=sample(sequence[i].timestamp_);PropagateInterval(healthy,a,b,b.timestamp_);}
    require(inner.ExportSnapshot().covariance(6,6)>healthy.ExportSnapshot().covariance(6,6),"held gyro uncertainty not increased");
    auto merged=MergeImuQuality(sequence[2].quality_,sequence[3].quality_);
    require(merged.held_mask==1 && merged.gyro_noise_scale.x()==100,"interval silently healthy");
    ESKFD raw_gap;raw_gap.SetInitialTime(1);require(!PropagateInterval(raw_gap,sample(1),sample(1.2),1.005),"partial boundary hides raw gap");
    ImuQualityGate expired(options);s=sample(1);expired.Process(s);
    for(int i=1;i<=5;++i){s=sample(1+i*.005);s.gyro_.x()=2;require(expired.Process(s)==Decision::Forward,"premature hold expiry");}
    s=sample(1.030);s.gyro_.x()=2;require(expired.Process(s)==Decision::Fault,"unbounded hold");
    ImuQualityGate uninitialized(options);s=sample(1);s.acce_.z()=30;
    require(uninitialized.Process(s)==Decision::Fault,"hold before healthy reference");
    std::cout<<"PASS saturation axes, NaN, timing/reset/gap, default stop, bounded hold/recovery, interval quality union, late replay state/P equality, Q restoration\n";
} catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
