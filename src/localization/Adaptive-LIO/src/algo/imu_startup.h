#pragma once
#include "tools/imu.h"
#include <limits>

namespace zjloc {
class ImuStartupGate {
public:
    bool Ready(const IMU& sample) {
        if (ready_) return true;
        if (!std::isfinite(sample.timestamp_) || !sample.gyro_.allFinite() || !sample.acce_.allFinite()) {
            last_stamp_ = stable_since_ = std::numeric_limits<double>::quiet_NaN();
            return false;
        }
        const double interval = sample.timestamp_ - last_stamp_;
        if (!std::isfinite(last_stamp_) || interval <= 0 || interval > .05) {
            stable_since_ = sample.timestamp_;
        }
        last_stamp_ = sample.timestamp_;
        ready_ = sample.timestamp_ - stable_since_ >= 1.0;
        return ready_;
    }
private:
    double last_stamp_ = std::numeric_limits<double>::quiet_NaN();
    double stable_since_ = std::numeric_limits<double>::quiet_NaN();
    bool ready_ = false;
};
}
