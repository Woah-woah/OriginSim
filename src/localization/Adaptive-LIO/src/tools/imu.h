//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_IMU_H
#define MAPPING_IMU_H

#include <memory>
#include <cstdint>
#include "common/eigen_types.h"

namespace zjloc
{

    struct ImuQuality {
        enum Reason : uint16_t { NonFinite=1, Saturated=2, Stale=4, TimeFault=8 };
        // bits 0..2 gyro, 3..5 accelerometer. Original quality is kept after substitution.
        uint8_t valid_mask=0x3f,saturation_mask=0,held_mask=0;
        uint16_t reasons=0;
        bool degraded=false;
        Vec3d gyro_noise_scale=Vec3d::Ones(),accel_noise_scale=Vec3d::Ones();
        double degraded_total_seconds=0.;
        bool Usable() const {
            return ((valid_mask|held_mask)&0x3f)==0x3f && gyro_noise_scale.allFinite() &&
                accel_noise_scale.allFinite() && (gyro_noise_scale.array()>=1).all() && (accel_noise_scale.array()>=1).all();
        }
    };
    /// IMU 读数
    struct IMU
    {
        IMU() = default;
        IMU(double t, const Vec3d &gyro, const Vec3d &acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

        ImuQuality quality_;
        double timestamp_ = 0.0;
        Vec3d gyro_ = Vec3d::Zero();
        Vec3d acce_ = Vec3d::Zero();
    };
} // namespace zjloc

using IMUPtr = std::shared_ptr<zjloc::IMU>;

#endif // MAPPING_IMU_H
