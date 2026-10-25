#pragma once
#include "tools/nav_state.h"
namespace zjloc {
// Copies every terminal navigation field; begin fields describe frozen prediction history.
template<class FrameState>
void CommitFrameState(const NavStated& posterior, FrameState& frame) {
    frame.rotation=posterior.R_.unit_quaternion(); frame.translation=posterior.p_;
    frame.velocity=posterior.v_; frame.ba=posterior.ba_; frame.bg=posterior.bg_;
}
template<class Points>
void RegisterCommittedPoints(const SE3& posterior, Points& points) {
    for (auto& p:points) p.point=posterior*p.end_point;
}
} // namespace zjloc
