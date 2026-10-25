#pragma once
#include "tools/imu.h"
#include <array>
#include <limits>
#include <stdexcept>
namespace zjloc {
struct ImuQualityOptions {
    Vec3d gyro_limit=Vec3d::Zero(); // rad/s, zero = unknown/disabled per axis
    Vec3d accel_limit=Vec3d::Zero(); // m/s^2 AFTER gnorm, zero = unknown/disabled
    double saturation_ratio=.95;
    double hold_seconds=0.; // default fail closed; experimental finite-axis hold only when explicitly enabled
    double noise_multiplier=100.; // variance multiplier on held channels, not a new baseline Q
    int recovery_samples=3;
    bool Valid() const {
        return gyro_limit.allFinite() && accel_limit.allFinite() && (gyro_limit.array()>=0).all() &&
            (accel_limit.array()>=0).all() && std::isfinite(saturation_ratio) && saturation_ratio>0 && saturation_ratio<=1 &&
            std::isfinite(hold_seconds) && hold_seconds>=0 && hold_seconds<=.05 &&
            std::isfinite(noise_multiplier) && noise_multiplier>=1 && recovery_samples>=1;
    }
};
class ImuQualityGate {
public:
    enum class Decision { Forward, Drop, Fault };
    explicit ImuQualityGate(ImuQualityOptions options={}) : options_(options) {
        if(!options.Valid())throw std::invalid_argument("Invalid IMU quality options (SI units; hold <=50 ms)");
        healthy_stamp_.fill(-std::numeric_limits<double>::infinity());
    }
    Decision Process(IMU& sample) {
        if(faulted_)return Decision::Fault;
        if(!std::isfinite(sample.timestamp_)){sample.quality_.reasons|=ImuQuality::NonFinite;return Decision::Drop;}
        const double dt=sample.timestamp_-last_stamp_;
        if(sample.timestamp_<last_stamp_-1. || (std::isfinite(last_stamp_) && dt>.05)) {
            sample.quality_.reasons|=ImuQuality::TimeFault;return Fault();
        }
        if(sample.timestamp_<=last_stamp_){sample.quality_.reasons|=ImuQuality::Stale;return Decision::Drop;}
        sample.quality_=ImuQuality{};
        for(int axis=0;axis<6;++axis) {
            double& value=axis<3?sample.gyro_[axis]:sample.acce_[axis-3];
            const double limit=axis<3?options_.gyro_limit[axis]:options_.accel_limit[axis-3];
            const bool finite=std::isfinite(value);
            const bool saturated=finite && limit>0 && std::abs(value)>=options_.saturation_ratio*limit;
            if(finite && !saturated) {healthy_[axis]=value;healthy_stamp_[axis]=sample.timestamp_;}
            else {
                sample.quality_.valid_mask &= ~(1u<<axis);
                if(saturated){sample.quality_.saturation_mask|=1u<<axis;sample.quality_.reasons|=ImuQuality::Saturated;}
                else sample.quality_.reasons|=ImuQuality::NonFinite;
                // No unlimited hold, no substitution before the first healthy value for that axis.
                if(options_.hold_seconds<=0 || !std::isfinite(healthy_stamp_[axis]) ||
                   sample.timestamp_-healthy_stamp_[axis]>options_.hold_seconds+1e-12) return Fault();
                value=healthy_[axis];sample.quality_.held_mask|=1u<<axis;
                if(axis<3)sample.quality_.gyro_noise_scale[axis]=options_.noise_multiplier;
                else sample.quality_.accel_noise_scale[axis-3]=options_.noise_multiplier;
            }
        }
        if(sample.quality_.held_mask) {
            if(!degraded_)degraded_start_=std::isfinite(last_stamp_)?last_stamp_:sample.timestamp_;
            degraded_=true;recovery_=0;
        } else if(degraded_) {
            if(++recovery_>=options_.recovery_samples)degraded_=false;
        }
        // Recovery hysteresis also consumes the finite episode budget.
        if(degraded_ && sample.timestamp_-degraded_start_>options_.hold_seconds+1e-12) return Fault();
        sample.quality_.degraded=degraded_;
        if(degraded_ && std::isfinite(last_stamp_))degraded_total_+=dt;
        sample.quality_.degraded_total_seconds=degraded_total_;
        last_stamp_=sample.timestamp_;return Decision::Forward;
    }
    bool Faulted() const{return faulted_;}
    double LastStamp() const{return last_stamp_;}
private:
    Decision Fault(){faulted_=true;return Decision::Fault;}
    ImuQualityOptions options_;
    std::array<double,6> healthy_{},healthy_stamp_;
    double last_stamp_=-std::numeric_limits<double>::infinity(),degraded_start_=0,degraded_total_=0;
    int recovery_=0;bool degraded_=false,faulted_=false;
};
// Intervals touching a degraded endpoint retain union masks and maximum noise scales.
// Even a right-end alpha=1 must not silently erase a bad left endpoint's interval quality.
inline ImuQuality MergeImuQuality(const ImuQuality& a,const ImuQuality& b) {
    ImuQuality result;
    result.valid_mask=a.valid_mask&b.valid_mask;
    result.saturation_mask=a.saturation_mask|b.saturation_mask;
    result.held_mask=a.held_mask|b.held_mask;result.reasons=a.reasons|b.reasons;
    result.degraded=a.degraded||b.degraded;
    result.gyro_noise_scale=a.gyro_noise_scale.cwiseMax(b.gyro_noise_scale);
    result.accel_noise_scale=a.accel_noise_scale.cwiseMax(b.accel_noise_scale);
    result.degraded_total_seconds=std::max(a.degraded_total_seconds,b.degraded_total_seconds);
    return result;
}
} // namespace zjloc
