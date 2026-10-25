#pragma once
#include "algo/eskf.hpp"
#include "algo/imu_quality.h"
#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>

namespace zjloc {
struct LioStateUpdate {
    ESKFD::Snapshot state;
    uint64_t sequence = 0;
    double last_accepted_lidar_stamp = 0;
    Vec3d gyro_measurement_variance = Vec3d::Constant(1e-5);
};

// One integration convention for both filters: use the right-end sample.
// For a LiDAR boundary inside a raw interval, interpolate the endpoint sample.
// A subsequent interval starts at the posterior time, never at the raw left time.
inline bool PropagateInterval(ESKFD& filter, const IMU& left, const IMU& right, double to) {
    const double from = filter.GetNominalState().timestamp_;
    if (!std::isfinite(to) || right.timestamp_ <= left.timestamp_ ||
        from < left.timestamp_-1e-9 || to > right.timestamp_+1e-9 || to <= from ||
        right.timestamp_-left.timestamp_>.05+1e-12 ||
        !left.quality_.Usable() || !right.quality_.Usable() ||
        !left.gyro_.allFinite() || !left.acce_.allFinite() ||
        !right.gyro_.allFinite() || !right.acce_.allFinite()) return false;
    const double alpha=std::clamp((to-left.timestamp_)/(right.timestamp_-left.timestamp_),0.0,1.0);
    IMU endpoint(to,(1-alpha)*left.gyro_+alpha*right.gyro_,(1-alpha)*left.acce_+alpha*right.acce_);
    endpoint.quality_=MergeImuQuality(left.quality_,right.quality_);
    return filter.Predict(endpoint);
}

// Single-owner core. Call only from the propagation worker (or a test thread).
class ImuOdomPredictor {
public:
    struct Counters {
        uint64_t invalid_imu=0, stale_imu=0, invalid_state=0, stale_state=0;
        uint64_t history_miss=0, gap=0, applied=0, timeout=0;
    } counters;
    explicit ImuOdomPredictor(double history_seconds=2.0, double prediction_limit=0.5)
        : history_seconds_(history_seconds), prediction_limit_(prediction_limit) {}

    bool Append(const IMU& imu) {
        if (!imu.quality_.Usable() || !std::isfinite(imu.timestamp_) || !imu.gyro_.allFinite() || !imu.acce_.allFinite()) {
            ++counters.invalid_imu; return false;
        }
        if (!history_.empty() && imu.timestamp_<=history_.back().timestamp_) {
            ++counters.stale_imu; return false;
        }
        history_.push_back(imu);
        // Keep the sample immediately before the history cutoff.
        while (history_.size()>2 && (history_[1].timestamp_<imu.timestamp_-history_seconds_ ||
                                    history_.size()>20000)) history_.pop_front();
        return true;
    }
    bool Submit(const LioStateUpdate& update) {
        // Runtime never silently resets just one estimator. A clock reset or gap
        // requires restarting the entire node.
        if (update.sequence==0 ||
            !ESKFD::ValidSnapshot(update.state) ||
            !std::isfinite(update.last_accepted_lidar_stamp) ||
            update.last_accepted_lidar_stamp>update.state.nav.timestamp_+1e-9 ||
            !update.gyro_measurement_variance.allFinite() ||
            (update.gyro_measurement_variance.array()<0).any()) {
            ++counters.invalid_state; return false;
        }
        if (update.sequence<=sequence_ || (pending_ && update.sequence<=pending_->sequence) ||
            (sequence_ && update.state.nav.timestamp_<=anchor_time_) ||
            (pending_ && update.state.nav.timestamp_<pending_->state.nav.timestamp_) ||
            (sequence_ && update.last_accepted_lidar_stamp<last_accepted_)) {
            ++counters.stale_state; return false;
        }
        pending_=update; return true;
    }
    void Advance() {
        if (faulted_ || history_.empty()) return;
        if (pending_ && pending_->state.nav.timestamp_<=history_.back().timestamp_) {
            const auto update=*pending_;
            pending_.reset();
            if (update.state.nav.timestamp_<history_.front().timestamp_) {
                ++counters.history_miss; Fail(); return;
            } else if (filter_.RestoreSnapshot(update.state)) {
                sequence_=update.sequence; anchor_time_=update.state.nav.timestamp_;
                last_accepted_=update.last_accepted_lidar_stamp;
                gyro_variance_=update.gyro_measurement_variance;
                initialized_=true; ++counters.applied;
            }
        }
        if (!initialized_) return;
        double t=filter_.GetNominalState().timestamp_;
        if (t<history_.front().timestamp_) { ++counters.history_miss; Fail(); return; }
        auto it=std::upper_bound(history_.begin(),history_.end(),t,
                                [](double time,const IMU& x){return time<x.timestamp_;});
        for (;it!=history_.end();++it) {
            if (it==history_.begin() ||
                !PropagateInterval(filter_,*std::prev(it),*it,it->timestamp_)) {
                ++counters.gap; Fail(); return;
            }
        }
    }
    bool Publishable() {
        if (faulted_ || !initialized_ || history_.empty()) return false;
        if (filter_.GetNominalState().timestamp_-last_accepted_>prediction_limit_) {
            ++counters.timeout; return false;
        }
        return true;
    }
    void Fail() { faulted_=true; initialized_=false; pending_.reset(); }
    bool Faulted() const { return faulted_; }
    ESKFD::Snapshot Snapshot() const { return filter_.ExportSnapshot(); }
    Vec3d AngularVelocity() const { return history_.back().gyro_-filter_.GetNominalState().bg_; }
    Vec3d GyroVariance() const { return history_.empty()?gyro_variance_:
        gyro_variance_.cwiseProduct(history_.back().quality_.gyro_noise_scale); }
    bool Degraded() const {return !history_.empty() && history_.back().quality_.degraded;}
    double DegradedSeconds() const {return history_.empty()?0.:history_.back().quality_.degraded_total_seconds;}
    size_t HistorySize() const { return history_.size(); }
    double LastAccepted() const { return last_accepted_; }
private:
    ESKFD filter_;
    std::deque<IMU> history_;
    std::optional<LioStateUpdate> pending_;
    uint64_t sequence_=0;
    double anchor_time_=0, last_accepted_=0;
    double history_seconds_, prediction_limit_;
    Vec3d gyro_variance_=Vec3d::Constant(1e-5);
    bool initialized_=false, faulted_=false;
};
} // namespace zjloc
