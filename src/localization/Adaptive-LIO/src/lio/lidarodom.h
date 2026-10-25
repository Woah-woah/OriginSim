#ifndef LIDAR_ODOM_M_H__
#define LIDAR_ODOM_M_H__
#include "common/timer/timer.h"
#include "lio/lidarFactor.h"
#include "algo/point_time.h"
#include "algo/frame_commit.h"
#include "algo/lidar_geometry_quality.h"
#include "algo/lidar_sampling.h"

#include "algo/eskf.hpp"
#include "algo/imu_odom_predictor.h"
#include "algo/match_quality.h"
#include "algo/aux_time_crop.h"
#include <atomic>
#include "algo/static_imu_init.h"
#include "lio/lio_utils.h"

#include "common/map.hpp"

#include <condition_variable>

#include "tools/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"
#include "common/utils.h"
#include "tools/point_types.h"

#include <sys/times.h>
// sys/vtimes.h is non-portable and not available on all systems (e.g., some Linux distributions).
// It was previously included for timing; we rely on common/timer/timer.h instead. Remove vtimes.h to avoid build errors.

namespace zjloc
{

     struct lioOptions_m
     {
          std::string lidar_covariance_mode="fixed";
          GeometryOptions geometry;
          double match_max_translation=2.0; // metres, hard same-time innovation limit
          double match_max_rotation=1.0; // radians, hard same-time innovation limit
          int diagnostic_stride=10;
          double surf_res;
          bool log_print;
          int max_num_iteration;
          //   ct_icp
          double size_voxel_map;
          double min_distance_points;
          int max_num_points_in_voxel;
          double max_distance;
          double weight_alpha;
          double weight_neighborhood;
          double max_dist_to_plane_icp;
          int init_num_frames;
          int voxel_neighborhood;
          int max_number_neighbors;
          int threshold_voxel_occupancy;
          bool estimate_normal_from_neighborhood;
          int min_number_neighbors;
          double power_planarity;
          int num_closest_neighbors;

          double sampling_rate;

          int max_num_residuals;
          int min_num_residuals;

          double thres_orientation_norm;
          double thres_translation_norm;
          int fov_segment_stride = 2;
          double sparse_scene_distance_thresh = 20.0;
          double sparse_nearest_neighbor_ratio = 3.0;
          double sparse_min_planarity = 0.15;
          double map_update_translation_trigger = 0.15;
          double map_update_rotation_trigger = 0.03;
          int map_update_max_skip_frames = 3;

          // Auxiliary LiDAR bundle fusion options.
          // When enabled, aux points are time-cropped to the main LiDAR scan,
          // transformed to the main LiDAR output frame, merged into the main
          // frame, and then used by the existing addSurfCostFactor() residual path.
          bool enable_aux_bundle_fusion = true;
          double aux_lidar_time_offset = 0.0; // corrected_aux_time = raw_aux_time + offset
          double aux_lidar_sync_margin = 0.01;

     };

     struct cloud_pub_m
     {
          float max_z_filter = 1.5f;
          float min_z_filter = -1.0f;
          float space_down_sample = 0.01f;
          bool enable_body_filter = false;
          float body_filter_x_min = -0.5f;
          float body_filter_x_max = 0.5f;
          float body_filter_y_min = -0.5f;
          float body_filter_y_max = 0.5f;
          float body_filter_z_min = -1.0f;
          float body_filter_z_max = 0.5f;
     };
     

     class lidarodom_m
     {
          friend struct LidarOdomTestAccess; // deterministic integration tests exercise real frame transactions
     public:
          lidarodom_m(/* args */);
          ~lidarodom_m();

          bool init(const std::string &config_yaml);

          void pushData(std::vector<point3D> &&, std::pair<double, double> data,bool is_aux,
                        std::chrono::steady_clock::time_point ingress_time = std::chrono::steady_clock::now());
          void pushData(IMUPtr imu);

          void run();
          void RequestStop() { stopping_.store(true); cond.notify_all(); }
          void Fail() { if(fault_callback_)fault_callback_(); RequestStop(); }
          void SetFaultCallback(std::function<void()> callback) { fault_callback_=std::move(callback); }
          void SetStateCallback(std::function<void(const LioStateUpdate&)> callback) { state_callback_=std::move(callback); }

          int getIndex() { return index_frame; }

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) { pub_cloud_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) { pub_data_to_ros = fun; }
          void setFunc(std::function<bool(const CloudPtr &cloud, const SE3 &pose, double time)> &fun) { pub_scantext_data = fun; }

     private:
          void loadOptions();
          /// 使用IMU初始化
          void TryInitIMU();

          /// 利用IMU预测状态信息
          /// 这段时间的预测数据会放入imu_states_里
          /// @return false when the current LiDAR frame crosses an IMU gap
          bool Predict();

          /// 对measures_中的点云去畸变
          void Undistort(std::vector<point3D> &points);

          std::vector<MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(MeasureGroup &meas);

          void stateInitialization();

          cloudFrame *buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                 double timestamp_begin, double timestamp_end);

          void poseEstimation(cloudFrame *p_frame, cloudFrame *p_frame_aux_publish_only);

          bool optimize(cloudFrame *p_frame);

          void lasermap_fov_segment();

          void map_incremental(cloudFrame *p_frame, cloudFrame *p_frame_aux_publish_only,
                               bool update_map, int min_num_points = 0);

          void publishFrameProducts(const SE3 &pose_of_lo, double stamp,
                                    const cloudFrame *p_frame_aux_publish_only);

          void addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                             double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points);

          void addSurfCostFactor(std::vector<ceres::CostFunction *> &surf,
                                 std::vector<point3D> &keypoints, const cloudFrame *p_frame);

          // search neighbors
          Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr,
                          double max_sq_distance = -1.0);

          inline Sophus::SO3d r2SO3(const Eigen::Vector3d r)
          {
               return Sophus::SO3d::exp(r);
          }

     private:
          /// @brief 数据
          std::string config_yaml_;
          size_t aux_point_cursor_ = 0; // 辅助点云的索引
          StaticIMUInit imu_init_;    // IMU静止初始化
          SE3 TIL_;                   //   lidar 转换到 imu
          MeasureGroup measures_;     // sync IMU and lidar scan
          bool imu_need_init_ = true; // 是否需要估计IMU初始零偏
          std::atomic<int> index_frame{1};
          std::atomic<bool> stopping_{false};
          lioOptions_m options_{};
          cloud_pub_m cloud_pub_options;
          size_t min_aux_points_ = 100;
          Eigen::Matrix3d R_main_aux_ = Eigen::Matrix3d::Identity();
          Eigen::Vector3d t_main_aux_ = Eigen::Vector3d::Zero();


          voxelHashMap voxel_map;

          Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); //   lidar 转换到 imu坐标系下
          Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();     //   need init

          /// @brief 滤波器
          ESKFD eskf_;
          std::function<void(const LioStateUpdate&)> state_callback_;
          std::function<void()> fault_callback_;
          uint64_t state_sequence_=0;
          double last_accepted_lidar_stamp_=0;
          Vec3d gyro_measurement_variance_=Vec3d::Constant(1e-5);
          MatchQuality match_quality_;
          LidarGeometryQuality geometry_quality_;
          SamplingCoverage sampling_coverage_;
          std::vector<LidarCorrespondence> final_correspondences_;
          double match_max_innovation_score_=100.0;
          bool prediction_fault_=false;
          std::vector<NavStated> imu_states_; // ESKF预测期间的状态
          IMUPtr last_imu_ = nullptr;

          double delay_time_;
          Vec3d g_{0, 0, -9.8};

          /// @brief 数据管理及同步
          std::deque<std::vector<point3D>> lidar_buffer_;
          std::deque<std::vector<point3D>> aux_lidar_buffer_;

          std::deque<IMUPtr> imu_buffer_;    // imu数据缓冲
          double last_timestamp_imu_ = -1.0; // 最近imu时间
          double last_timestamp_lidar_ = 0;  // 最近lidar时间
          std::deque<std::pair<double, double>> time_buffer_;
          std::deque<std::chrono::steady_clock::time_point> ingress_time_buffer_;
          std::deque<std::pair<double, double>> aux_lidar_time_buffer_;


          /// @brief mutex
          std::mutex mtx_buf;
          std::condition_variable cond;

          state *current_state;

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, double time1, double time2)> pub_data_to_ros;
          // Callback for ScanContext module (world cloud + world pose)
          std::function<bool(const CloudPtr &cloud, const SE3 &pose, double time)> pub_scantext_data;

          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
          bool has_last_map_maintenance_pose_ = false;
          Eigen::Vector3d last_map_maintenance_translation_ = Eigen::Vector3d::Zero();
          Eigen::Quaterniond last_map_maintenance_rotation_ = Eigen::Quaterniond::Identity();
          int last_map_maintenance_frame_ = 0;
     };
}

#endif // LIDAR_ODOM_M_H__
