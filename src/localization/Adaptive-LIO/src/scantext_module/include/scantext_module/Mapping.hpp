#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <fstream>
#include "scantext_module/ScanContext.hpp"

namespace scantext
{

    /**
     * @brief KeyFrame structure for the map
     */
    struct KeyFrame
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int index;
        double time;
        Eigen::Isometry3d pose;
        std::string cloud_path;
        std::string descriptor_cache_path;
        ScanContext::PointCloudType::Ptr cloud;
        ScanContext::SCDescriptor descriptor;
        ScanContext::RingKey ring_key;
        ScanContext::CartDescriptor cart_descriptor;
    };

    /**
     * @brief Mapping Core for managing KeyFrames and Map saving
     */
    class MappingCore
    {
    public:
        struct Config
        {
            bool enable_mapping = false;
            double keyframe_dist_thresh = 1.0;  // Meters
            double keyframe_angle_thresh = 0.2; // Radians
            std::string map_save_path = "/tmp/map.pcd";
            double auto_save_interval = 0.0; // Meters, 0 to disable
        };

        MappingCore();
        ~MappingCore() = default;

        void setConfig(const Config &config);
        void setScanContextParams(const SCParams &params);
        Config getConfig() const { return config_; }


        /**
         * @brief Save the global map to a PCD file
         * @param path File path
         * @return true Success
         */
        bool saveMap(const std::string &path);

        /**
         * @brief Save the global map to a PCD file asynchronously
         * @param path File path
         * @param db_dir Directory to save the database to
         * @return true Success
         */
        void saveMapAsync(const std::string &path);

        void initDatabaseIfNeeded();

        /**
         * @brief Save the database (poses + individual clouds + descriptors)
         * @param directory Directory to save to
         * @return true Success
         */
        bool saveDatabase(const std::string &directory);

        // Get all keyframes (thread safe)
        std::vector<std::shared_ptr<KeyFrame>> getKeyFrames();

        bool addFrameWithSC(const ScanContext::PointCloudType::Ptr &cloud,
                            const Eigen::Isometry3d &pose,
                            double time,
                            const ScanContext::SCDescriptor &sc,
                            const ScanContext::RingKey &rk,
                            std::shared_ptr<KeyFrame> *out_kf = nullptr);

    private:
        Config config_;
        ScanContext scan_context_;

        std::vector<std::shared_ptr<KeyFrame>> keyframes_;
        std::mutex map_mutex_;

        Eigen::Isometry3d last_keyframe_pose_ = Eigen::Isometry3d::Identity();
        double accumulated_dist_since_save_ = 0.0;

        // Helper to check if we should create a keyframe
        bool isKeyFrame(const Eigen::Isometry3d &pose);

        std::atomic<bool> saving_{false};
        ScanContext::PointCloudType::Ptr global_map_;

        std::ofstream pose_ofs_;
        std::string db_dir_;
        bool db_initialized_{false};

        int frame_index_{0};
    };

} // namespace scantext
