#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <memory>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace scantext
{

    // ScanContext parameters
    struct SCParams
    {
        int num_ring = 20;
        int num_sector = 60;
        double max_radius = 80.0;
        double lidar_height = 2.0; // For filtering ground if needed, or just relative
        bool use_scpp = true;
        double scpp_search_ratio = 0.1;

        // Cart Context (Scan Context++ TRO)
        double cart_x_unit = 5.0;
        double cart_y_unit = 2.0;
        double cart_x_max = 100.0;
        double cart_y_max = 40.0;
    };

    /**
     * @brief ScanContext descriptor extractor and matcher
     */
    class ScanContext
    {
    public:
        using PointType = pcl::PointXYZI;
        using PointCloudType = pcl::PointCloud<PointType>;
        using SCDescriptor = Eigen::MatrixXd; // num_ring x num_sector
        using RingKey = Eigen::VectorXd;      // num_ring x 1
        using SectorKey = Eigen::VectorXd;    // num_sector x 1
        using CartDescriptor = Eigen::MatrixXd;
        template <typename PointT, typename GetXYZ>
        SCDescriptor makeScanContextFromPoints(const std::vector<PointT> &points, GetXYZ get_xyz) const;

        /**
         * @brief Construct a new Scan Context object
         *
         * @param params Parameters for grid generation
         */
        ScanContext(const SCParams &params = SCParams());
        ~ScanContext() = default;

        /**
         * @brief Extract ScanContext descriptor from a point cloud
         *
         * @param scan Input point cloud (should be undistorted and roughly horizontal)
         * @return SCDescriptor (Matrix of num_ring x num_sector)
         */
        SCDescriptor makeScanContext(const PointCloudType &scan);

        /**
         * @brief Generate RingKey from ScanContext (for fast candidate search)
         *
         * @param sc Input descriptor
         * @return RingKey Vector of ring averages
         */
        RingKey makeRingKey(const SCDescriptor &sc);

        SectorKey makeSectorKey(const SCDescriptor &sc);

        CartDescriptor makeCartContext(const PointCloudType &scan, double yaw_offset = 0.0);

        double distanceBtnCartContext(const CartDescriptor &cc1, const CartDescriptor &cc2) const;

        /**
         * @brief Calculate distance between two ScanContexts
         *
         * Performs a shift search to find the best alignment (rotation invariance).
         *
         * @param sc1 Reference descriptor
         * @param sc2 Query descriptor
         * @return std::pair<double, int> {distance, best_shift_index}
         */
        std::pair<double, int> distanceBtnScanContext(const SCDescriptor &sc1, const SCDescriptor &sc2);

        // Get parameters
        const SCParams &getParams() const { return params_; }

    private:
        SCParams params_;

        // Helper: circular shift of matrix columns
        SCDescriptor circshift(const SCDescriptor &sc, int shift);

        int fastAlignUsingSectorKey(const SectorKey &vkey_ref, const SectorKey &vkey_query) const;

        double distDirectSC(const SCDescriptor &sc1, const SCDescriptor &sc2) const;
    };


    template <typename PointT, typename GetXYZ>
    ScanContext::SCDescriptor ScanContext::makeScanContextFromPoints(
        const std::vector<PointT> &points, GetXYZ get_xyz) const
    {
        const int num_ring = params_.num_ring;
        const int num_sector = params_.num_sector;
        const double max_radius = params_.max_radius;

        const double gap_ring = max_radius / static_cast<double>(num_ring);
        const double gap_sector = 2.0 * M_PI / static_cast<double>(num_sector);

        SCDescriptor sc_desc = Eigen::MatrixXd::Zero(num_ring, num_sector);

        for (const auto &p : points)
        {
            // get_xyz(p) 需要返回一个有 x()/y()/z() 的类型（Eigen::Vector3d/Vec3d 都行）
            auto v = get_xyz(p);
            const double x = static_cast<double>(v.x());
            const double y = static_cast<double>(v.y());
            const double z = static_cast<double>(v.z());

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            const double range = std::sqrt(x * x + y * y);
            if (range > max_radius)
                continue;

            double angle = std::atan2(y, x); // [-pi, pi]
            if (angle < 0)
                angle += 2.0 * M_PI; // [0, 2pi)

            const int idx_ring = std::min(std::max(static_cast<int>(range / gap_ring), 0), num_ring - 1);
            const int idx_sector = std::min(std::max(static_cast<int>(angle / gap_sector), 0), num_sector - 1);

            // 取 bin 内最大高度
            const double &cur = sc_desc(idx_ring, idx_sector);
            if (cur == 0.0)
                sc_desc(idx_ring, idx_sector) = z;
            else
                sc_desc(idx_ring, idx_sector) = std::max(cur, z);
        }

        return sc_desc;
    }

} // namespace scantext
