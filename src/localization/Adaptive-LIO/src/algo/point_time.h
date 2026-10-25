#pragma once
#include "tools/nav_state.h"
#include <algorithm>
#include <vector>

namespace zjloc {
// Trajectory must be finite, strictly ordered and cover the sample. No extrapolation.
// Output is q^{I_end}; original sample coordinates and sensor timestamp are immutable.
inline bool DeskewToEndImu(const Vec3d& raw, double stamp,
                          const std::vector<NavStated>& trajectory,
                          const SE3& T_IL, const SE3& T_WI_end, Vec3d& result) {
    if (!raw.allFinite() || !std::isfinite(stamp) || trajectory.empty() ||
        stamp<trajectory.front().timestamp_ || stamp>trajectory.back().timestamp_ ||
        !T_IL.matrix().allFinite() || !T_WI_end.matrix().allFinite()) return false;
    auto right=std::lower_bound(trajectory.begin(),trajectory.end(),stamp,
        [](const NavStated& s,double t){return s.timestamp_<t;});
    if (right==trajectory.end()) return false;
    SE3 pose=right->GetSE3();
    if (right->timestamp_!=stamp) {
        if (right==trajectory.begin()) return false;
        const auto& left=*std::prev(right);
        const double dt=right->timestamp_-left.timestamp_;
        if (!(dt>0)) return false;
        const double a=(stamp-left.timestamp_)/dt;
        pose=SE3(left.R_.unit_quaternion().slerp(a,right->R_.unit_quaternion()),
                 (1-a)*left.p_+a*right->p_);
    }
    const Vec3d end=T_WI_end.inverse()*pose*T_IL*raw;
    if (!end.allFinite()) return false;
    result=end;
    return true;
}
} // namespace zjloc
