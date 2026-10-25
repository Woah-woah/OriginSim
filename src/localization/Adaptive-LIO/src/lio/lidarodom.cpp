#include "common/config.hpp"
#include "lidarodom.h"
#include <glog/logging.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <ceres/ceres.h>
// #include <ceres/local_parameterization.h>
#include <ceres/manifold.h>
#include <ceres/rotation.h>
#include <algorithm>
#include <cmath>

#include "common/math_utils.h"
#include "common/cloudMap.hpp"
#include "tools/point_types.h"

namespace zjloc
{

     namespace
     {
          Eigen::Quaterniond NormalizeQuaternion(Eigen::Quaterniond q)
          {
               if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
                   !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
                   q.norm() < 1e-12)
               {
                    return Eigen::Quaterniond::Identity();
               }
               q.normalize();
               return q;
          }

          SE3 SafeSE3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t)
          {
               return SE3(NormalizeQuaternion(q), t);
          }

          Eigen::Matrix3d RpyToMatrixRad(double roll, double pitch, double yaw)
          {
               const Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
               const Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
               const Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
               return (Rz * Ry * Rx).toRotationMatrix();
          }

          Eigen::Vector3d Vec3FromYaml(const YAML::Node &node, const Eigen::Vector3d &fallback)
          {
               if (!node || !node.IsSequence() || node.size() < 3)
               {
                    return fallback;
               }
               return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
          }
     }

     lidarodom_m::lidarodom_m(/* args */)
     {
          current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
          points_world->points.reserve(20000);
     }

     lidarodom_m::~lidarodom_m()
     {
          delete current_state;
     }

     void lidarodom_m::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);

          auto node = GetNode(config_yaml_);

          if (node["map_options"]) throw std::runtime_error("Retired map_options: active map uses odometry.size_voxel_map");
          if (node["odometry"])
          {
               auto odometry_node = node["odometry"];
               OPTION_CLAUSE(odometry_node, options_, surf_res, double);
               OPTION_CLAUSE(odometry_node, options_, log_print, bool);
               OPTION_CLAUSE(odometry_node, options_, max_num_iteration, int);

               OPTION_CLAUSE(odometry_node, options_, size_voxel_map, double);
               OPTION_CLAUSE(odometry_node, options_, min_distance_points, double);
               OPTION_CLAUSE(odometry_node, options_, max_num_points_in_voxel, int);
               OPTION_CLAUSE(odometry_node, options_, max_distance, double);
               OPTION_CLAUSE(odometry_node, options_, weight_alpha, double);
               OPTION_CLAUSE(odometry_node, options_, weight_neighborhood, double);
               OPTION_CLAUSE(odometry_node, options_, max_dist_to_plane_icp, double);
               OPTION_CLAUSE(odometry_node, options_, init_num_frames, int);
               OPTION_CLAUSE(odometry_node, options_, voxel_neighborhood, int);
               OPTION_CLAUSE(odometry_node, options_, max_number_neighbors, int);
               OPTION_CLAUSE(odometry_node, options_, threshold_voxel_occupancy, int);
               OPTION_CLAUSE(odometry_node, options_, estimate_normal_from_neighborhood, bool);
               OPTION_CLAUSE(odometry_node, options_, min_number_neighbors, int);
               OPTION_CLAUSE(odometry_node, options_, power_planarity, double);
               OPTION_CLAUSE(odometry_node, options_, num_closest_neighbors, int);

               OPTION_CLAUSE(odometry_node, options_, sampling_rate, double);
               OPTION_CLAUSE(odometry_node, options_, max_num_residuals, int);
               OPTION_CLAUSE(odometry_node, options_, min_num_residuals, int);

               for (const char* retired : {"icpmodel", "motion_compensation", "point_to_plane_with_distortion",
                                            "beta_location_consistency", "beta_orientation_consistency",
                                            "beta_constant_velocity", "beta_small_velocity"}) {
                    if (odometry_node[retired]) throw std::runtime_error(std::string("Retired odometry parameter: ")+retired+
                        "; migrate to point_time_model: IMU_DESKEW_END");
               }
               if (!odometry_node["point_time_model"] ||
                   odometry_node["point_time_model"].as<std::string>() != "IMU_DESKEW_END")
                    throw std::runtime_error("point_time_model must be IMU_DESKEW_END");
                OPTION_CLAUSE(odometry_node, options_, thres_orientation_norm, double);
                OPTION_CLAUSE(odometry_node, options_, thres_translation_norm, double);
                OPTION_CLAUSE(odometry_node, options_, fov_segment_stride, int);
                OPTION_CLAUSE(odometry_node, options_, sparse_scene_distance_thresh, double);
                OPTION_CLAUSE(odometry_node, options_, sparse_nearest_neighbor_ratio, double);
                OPTION_CLAUSE(odometry_node, options_, sparse_min_planarity, double);
                OPTION_CLAUSE(odometry_node, options_, map_update_translation_trigger, double);
                OPTION_CLAUSE(odometry_node, options_, map_update_rotation_trigger, double);
                OPTION_CLAUSE(odometry_node, options_, map_update_max_skip_frames, int);
                OPTION_CLAUSE(odometry_node, options_, enable_aux_bundle_fusion, bool);
                OPTION_CLAUSE(odometry_node, options_, aux_lidar_time_offset, double);
                OPTION_CLAUSE(odometry_node, options_, aux_lidar_sync_margin, double);
                OPTION_CLAUSE(odometry_node, options_, lidar_covariance_mode, std::string);
                OPTION_CLAUSE(odometry_node, options_, match_max_translation, double);
                OPTION_CLAUSE(odometry_node, options_, match_max_rotation, double);
                OPTION_CLAUSE(odometry_node, options_, diagnostic_stride, int);
                auto geometry=odometry_node["geometry"];
                if(geometry) {
                    if(geometry["reference_length_m"]) options_.geometry.reference_length=geometry["reference_length_m"].as<double>();
                    if(geometry["variance_floor"]) options_.geometry.variance_floor=geometry["variance_floor"].as<double>();
                    if(geometry["variance_ceiling"]) options_.geometry.variance_ceiling=geometry["variance_ceiling"].as<double>();
                    if(geometry["residual_variance_m2"]) options_.geometry.residual_variance=geometry["residual_variance_m2"].as<double>();
                    if(geometry["correlation_inflation"]) options_.geometry.correlation_inflation=geometry["correlation_inflation"].as<double>();
                }
                if ((options_.lidar_covariance_mode!="fixed" && options_.lidar_covariance_mode!="adaptive") ||
                    !options_.geometry.Valid() || !std::isfinite(options_.match_max_translation) || options_.match_max_translation<=0 ||
                    !std::isfinite(options_.match_max_rotation) || options_.match_max_rotation<=0 || options_.match_max_rotation>=M_PI ||
                    options_.diagnostic_stride<1 || options_.min_num_residuals<1 || options_.max_num_residuals<options_.min_num_residuals || options_.max_num_residuals>100000 ||
                    options_.max_num_iteration<1 || options_.min_number_neighbors<3 || options_.num_closest_neighbors<1 ||
                    options_.num_closest_neighbors>options_.min_number_neighbors ||
                    !std::isfinite(options_.weight_alpha+options_.weight_neighborhood) ||
                    std::abs(options_.weight_alpha)+std::abs(options_.weight_neighborhood)<=0)
                     throw std::runtime_error("Invalid LiDAR covariance/geometry/matching options");
                if(options_.lidar_covariance_mode=="adaptive") LOG(WARNING)<<"adaptive R is experimental and requires independent calibration";

          }
          if (node["cloud_pub"])
          {
               auto pub_node = node["cloud_pub"];
               OPTION_CLAUSE(pub_node, cloud_pub_options, max_z_filter, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, min_z_filter, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, space_down_sample, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, enable_body_filter, bool);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_x_min, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_x_max, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_y_min, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_y_max, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_z_min, float);
               OPTION_CLAUSE(pub_node, cloud_pub_options, body_filter_z_max, float);
          }
     }

     bool lidarodom_m::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;
          StaticIMUInit::Options imu_init_options;
          imu_init_options.use_speed_for_static_checking_ = false; // 本节数据不需要轮速计
          imu_init_ = StaticIMUInit(imu_init_options);

          auto yaml = YAML::LoadFile(config_yaml_);
          delay_time_ = yaml["delay_time"].as<double>();
          if (!std::isfinite(delay_time_) || delay_time_<0) throw std::runtime_error("delay_time must be finite and nonnegative");
          if (yaml["odometry"]["match_max_innovation_score"])
               match_max_innovation_score_=yaml["odometry"]["match_max_innovation_score"].as<double>();
          if (!std::isfinite(match_max_innovation_score_) || match_max_innovation_score_<=0)
               throw std::runtime_error("match_max_innovation_score must be positive");
          if (yaml["aux_lidar"])
          {
               auto aux_node = yaml["aux_lidar"];
               if (aux_node["min_points_per_measurement"])
               {
                    min_aux_points_ = aux_node["min_points_per_measurement"].as<size_t>();
               }
               if (aux_node["enable_bundle_fusion"])
               {
                    options_.enable_aux_bundle_fusion = aux_node["enable_bundle_fusion"].as<bool>();
               }
               if (aux_node["time_offset"])
               {
                    options_.aux_lidar_time_offset = aux_node["time_offset"].as<double>();
               }
               if (aux_node["sync_margin"])
               {
                    options_.aux_lidar_sync_margin = aux_node["sync_margin"].as<double>();
               }

               t_main_aux_ = Vec3FromYaml(aux_node["extrinsic_xyz"], Eigen::Vector3d::Zero());
               Eigen::Vector3d rpy_rad = Eigen::Vector3d::Zero();
               if (aux_node["extrinsic_rpy_deg"])
               {
                    rpy_rad = Vec3FromYaml(aux_node["extrinsic_rpy_deg"], Eigen::Vector3d::Zero()) * M_PI / 180.0;
               }
               else if (aux_node["extrinsic_rpy"])
               {
                    rpy_rad = Vec3FromYaml(aux_node["extrinsic_rpy"], Eigen::Vector3d::Zero());
               }
               R_main_aux_ = RpyToMatrixRad(rpy_rad.x(), rpy_rad.y(), rpy_rad.z());
          }
          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          q_IL = NormalizeQuaternion(q_IL);
          lidar_R_wrt_IMU = q_IL.toRotationMatrix();
          // init TIL
          TIL_ = SafeSE3(q_IL, lidar_T_wrt_IMU);
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;
          std::cout << "aux bundle fusion: " << (options_.enable_aux_bundle_fusion ? "enabled" : "disabled")
                    << ", aux_time_offset=" << options_.aux_lidar_time_offset
                    << ", aux_sync_margin=" << options_.aux_lidar_sync_margin << std::endl;
          std::cout << "T_main_aux R:\n" << R_main_aux_ << "\nt:" << t_main_aux_.transpose() << std::endl;

          loadOptions();
          return true;
     }

     void lidarodom_m::pushData(std::vector<point3D> &&msg, std::pair<double, double> data, bool is_aux,
                               std::chrono::steady_clock::time_point ingress_time)
     {
          // Gazebo 快照的起止时间相同；负时长和非有限时间仍直接拒绝。
          if (!std::isfinite(data.first) || !std::isfinite(data.second) || data.second < 0.0)
          {
               LOG(ERROR) << std::setprecision(18)
                          << "drop lidar with invalid time [begin=" << data.first
                          << ", duration=" << data.second << "]";
               return;
          }
          msg.erase(std::remove_if(msg.begin(),msg.end(),[](const point3D& p){
               return !std::isfinite(p.timestamp)||!p.raw_point.allFinite();
          }),msg.end());
          if (is_aux) SortFinitePoints(msg);
          {
               // getMeasureMents() reads all lidar/imu queues while holding mtx_buf.
               // Use the same mutex for writes to avoid data races under high-rate callbacks.
               std::lock_guard<std::mutex> lk(mtx_buf);
               if (stopping_) return;
               if (is_aux)
               {
                    if (!aux_lidar_time_buffer_.empty() && data.first<=aux_lidar_time_buffer_.back().first) {
                         LOG(WARNING)<<"drop out-of-order aux packet"; return;
                    }
                    // Aux is optional: discard oldest complete packets on backlog.
                    if(aux_lidar_buffer_.size()>=200){aux_lidar_buffer_.pop_front();aux_lidar_time_buffer_.pop_front();aux_point_cursor_=0;}
                    aux_lidar_buffer_.push_back(std::move(msg));
                    aux_lidar_time_buffer_.push_back(data);
               }
               else
               {
                    if (data.first <= last_timestamp_lidar_)
                    {
                         // Fail closed: accepting an old scan would make the ESKF
                         // interpolate backwards and can move the map. A real clock
                         // reset requires an explicit estimator reset/restart.
                         LOG(ERROR) << std::setprecision(18)
                                    << "drop non-monotonic lidar [stamp=" << data.first
                                    << ", last=" << last_timestamp_lidar_ << "]";
                         return;
                    }

                    // 实时定位只保留最新待处理帧；IMU 队列保留，预测仍覆盖完整时间段。
                    if (!lidar_buffer_.empty())
                    {
                         lidar_buffer_.pop_front();
                         time_buffer_.pop_front();
                         ingress_time_buffer_.pop_front();
                    }
                    lidar_buffer_.push_back(std::move(msg));
                    time_buffer_.push_back(data);
                    ingress_time_buffer_.push_back(ingress_time);
                    last_timestamp_lidar_ = data.first;
               }
          }
          cond.notify_one();
     }
     void lidarodom_m::pushData(IMUPtr imu)
     {
          if (imu == nullptr)
          {
               LOG(ERROR) << "drop null imu";
               return;
          }
          const double timestamp = imu->timestamp_;
          {
               std::lock_guard<std::mutex> lk(mtx_buf);
               if (!std::isfinite(timestamp))
               {
                    LOG(ERROR) << "drop imu with non-finite timestamp";
                    return;
               }
               if (!imu->gyro_.allFinite() || !imu->acce_.allFinite())
               {
                    LOG(ERROR) << std::setprecision(18)
                               << "drop imu with non-finite measurement [stamp=" << timestamp << "]";
                    return;
               }
               if (timestamp <= last_timestamp_imu_)
               {
                    // Never clear good buffered samples or lower the timestamp
                    // watermark because of one delayed/duplicated UDP packet.
                    LOG(WARNING) << std::setprecision(18)
                                 << "drop non-monotonic imu before buffering [stamp=" << timestamp
                                 << ", last=" << last_timestamp_imu_ << "]";
                    return;
               }

               if(stopping_) return;
               if(imu_buffer_.size()>=20000){LOG(ERROR)<<"LIO IMU queue overflow; restart node";Fail();return;}
               last_timestamp_imu_ = timestamp;
               imu_buffer_.emplace_back(imu);
          }
          cond.notify_one();
     }

     void lidarodom_m::run()
     {
          while (!stopping_)
          {
               std::vector<MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               cond.wait(lk, [&]
                         { return stopping_ || (measurements = getMeasureMents()).size() != 0; });
               if (stopping_) break;
               lk.unlock();

               for (auto &m : measurements)
               {
                    if(stopping_)break;
                    zjloc::common::Timer::Evaluate([&]()
                                                   { ProcessMeasurements(m); },
                                                   "processMeasurement");
                    const auto process_end = std::chrono::steady_clock::now();
                    // One sample per processed main-LiDAR frame, starting at callback entry.
                    zjloc::common::Timer::Record("frame_callback_to_done",
                         std::chrono::duration<double, std::milli>(process_end - m.ingress_time_).count());

                    {
                         auto real_time = std::chrono::steady_clock::now();
                         static std::chrono::steady_clock::time_point prev_real_time = real_time;

                         if (real_time - prev_real_time > std::chrono::seconds(5))
                         {
                              auto data_time = m.lidar_end_time_;
                              static double prev_data_time = data_time;
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              auto delta_sim = data_time - prev_data_time;
                              printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     void lidarodom_m::ProcessMeasurements(MeasureGroup &meas)
     {
          measures_.lidar_begin_time_ = meas.lidar_begin_time_;
          measures_.lidar_end_time_ = meas.lidar_end_time_;
          measures_.imu_ = meas.imu_;
          measures_.lidar_.clear();
          measures_.aux_lidar_.clear();

          if (imu_need_init_)
          {
               TryInitIMU();
               return;
          }

          // std::cout << ANSI_DELETE_LAST_LINE;
          if (options_.log_print)
          {
               std::cout << ANSI_COLOR_GREEN << "============== process frame: "
                         << index_frame << ANSI_COLOR_RESET << std::endl;
          }
          imu_states_.clear(); //   need clear here

          // 利用IMU数据进行状态预测
          bool imu_prediction_valid = false;
          zjloc::common::Timer::Evaluate([&]()
                                         { imu_prediction_valid = Predict(); },
                                         "predict");
          if (!imu_prediction_valid)
          {
               // No state-clock jump across a missing interval. Fail closed;
               // restart the whole node rather than resetting only one estimator.
               LOG(ERROR) << std::setprecision(18)
                          << "drop lidar frame because IMU prediction is incomplete [begin="
                          << meas.lidar_begin_time_ << ", end=" << meas.lidar_end_time_ << "]";
               prediction_fault_=true;
               Fail();
               return;
          }
          //
          zjloc::common::Timer::Evaluate([&]()
                                         { stateInitialization(); },
                                         "state init");

          std::vector<point3D> const_surf = std::move(meas.lidar_);
          std::vector<point3D> const_surf_aux = std::move(meas.aux_lidar_);

          cloudFrame *p_frame = nullptr;
          cloudFrame *p_frame_aux_publish_only = nullptr;

          if (!const_surf_aux.empty())
          {
               const double frame_dt = std::max(1e-6, meas.lidar_end_time_ - meas.lidar_begin_time_);
               for (auto &aux_point : const_surf_aux)
               {
                    // Auxiliary extrinsic convention:
                    //   p_main = R_main_aux_ * p_aux + t_main_aux_
                    // Apply it in both modes. When bundle fusion is disabled, these
                    // transformed points are used by the /livox/scan publication path
                    // only and never enter optimization or the voxel map.
                    const Eigen::Vector3d p_main = R_main_aux_ * aux_point.raw_point + t_main_aux_;
                    aux_point.raw_point = p_main;
                    aux_point.point = p_main;
                    aux_point.lid = 1;

                    aux_point.relative_time = aux_point.timestamp - meas.lidar_begin_time_;
                    aux_point.timespan = frame_dt;
                    aux_point.alpha_time = std::clamp(aux_point.relative_time / frame_dt, 0.0, 1.0);
               }

               if (options_.enable_aux_bundle_fusion)
               {
                    const_surf.reserve(const_surf.size() + const_surf_aux.size());
                    for (auto &aux_point : const_surf_aux)
                    {
                         const_surf.emplace_back(std::move(aux_point));
                    }
                    std::vector<point3D>().swap(const_surf_aux);
               }
          }

          zjloc::common::Timer::Evaluate([&]()
                                         { p_frame = buildFrame(const_surf, current_state,
                                                                meas.lidar_begin_time_,
                                                                meas.lidar_end_time_); },
                                         "build frame");

          if (!options_.enable_aux_bundle_fusion && !const_surf_aux.empty())
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { p_frame_aux_publish_only = buildFrame(const_surf_aux, current_state,
                                                                                      meas.lidar_begin_time_,
                                                                                      meas.lidar_end_time_); },
                                              "build aux publish frame");
          }

          // Match first, validate and correct, only then maintain map/products.
          const auto prior=eskf_.ExportSnapshot();
          const bool seed=voxel_map.empty();
          match_quality_=MatchQuality{};
          geometry_quality_=LidarGeometryQuality{};
          sampling_coverage_=SamplingCoverage{};
          const auto match_start=std::chrono::steady_clock::now();
          bool accepted=false;
          double innovation_score=0;
          const char* reason=seed?"seed_insufficient_support":"solver_unusable";
          Mat6d observation_covariance=Mat6d::Identity()*.01;
          SE3 candidate=prior.nav.GetSE3();
          if(seed) {
               accepted=ESKFD::ValidSnapshot(prior) && p_frame->point_surf.size()>=static_cast<size_t>(options_.min_num_residuals);
          } else {
               bool optimized = false;
               zjloc::common::Timer::Evaluate([&]() { optimized = optimize(p_frame); }, "optimize");
               if (optimized) {
               candidate=SafeSE3(p_frame->p_state->rotation,p_frame->p_state->translation);
               if (options_.lidar_covariance_mode=="adaptive") {
                    const Mat6d candidate_covariance=BuildAdaptiveCovariance(geometry_quality_,options_.geometry);
                    observation_covariance=TransportCandidateCovariance(prior.nav.GetSE3(),candidate,candidate_covariance);
               }
               accepted=AcceptMatch(match_quality_,options_.min_num_residuals,prior,candidate,
                                    observation_covariance,match_max_innovation_score_,innovation_score,&reason,
                                    options_.match_max_translation,options_.match_max_rotation);
               if(accepted) {
                    zjloc::common::Timer::Evaluate([&]() {
                         accepted=eskf_.ObserveSE3(candidate,observation_covariance);
                         if(!accepted)reason="invalid_posterior";
                    }, "eskf_obs");
               }
               } else reason=match_quality_.reason;
          }
          if(seed && accepted)reason="seed";
          const auto posterior=eskf_.ExportSnapshot();
          if (!ESKFD::ValidSnapshot(posterior)) { Fail(); p_frame->release(); delete p_frame;
               if(p_frame_aux_publish_only){p_frame_aux_publish_only->release();delete p_frame_aux_publish_only;} return; }
          // Every cache derives from the authoritative filter, including rejection and seed.
          CommitFrameState(posterior.nav,*current_state);
          *p_frame->p_state=*current_state;
          if (p_frame_aux_publish_only) *p_frame_aux_publish_only->p_state=*current_state;
          if(accepted) {
               last_accepted_lidar_stamp_=meas.lidar_end_time_;
               RegisterCommittedPoints(posterior.nav.GetSE3(),p_frame->point_surf);
          }
          const double candidate_posterior_dp=(candidate.translation()-posterior.nav.p_).norm();
          const double candidate_posterior_dr=(posterior.nav.R_.inverse()*candidate.so3()).log().norm();
          // Value snapshot leaves the LIO thread before expensive map/cloud work.
          if(state_callback_)state_callback_({posterior,++state_sequence_,
              last_accepted_lidar_stamp_,gyro_measurement_variance_});
          const double match_ms=std::chrono::duration<double,std::milli>(
              std::chrono::steady_clock::now()-match_start).count();
          if(index_frame%options_.diagnostic_stride==0 || !accepted || seed) LOG(INFO)<<"lio_quality stamp="<<std::setprecision(16)<<meas.lidar_end_time_
                   <<" accepted="<<accepted<<" seed="<<seed<<" support="<<match_quality_.residuals
                   <<" cost_before="<<match_quality_.initial_cost<<" cost_after="<<match_quality_.final_cost
                   <<" reason="<<reason<<" geometry_rank="<<geometry_quality_.rank
                   <<" geometry_spectrum="<<geometry_quality_.eigenvalues.transpose()
                   <<" information_per_point="<<geometry_quality_.information_per_point
                   <<" candidate_count="<<sampling_coverage_.candidates<<" kept="<<sampling_coverage_.retained
                   <<" normal_bins="<<sampling_coverage_.normal_bins<<" spatial_bins="<<sampling_coverage_.spatial_bins
                   <<" normal_axis_counts="<<sampling_coverage_.dominant_normal_counts.transpose()
                   <<" R_mode="<<options_.lidar_covariance_mode
                   <<" R_spectrum="<<Eigen::SelfAdjointEigenSolver<Mat6d>(observation_covariance).eigenvalues().transpose()
                   <<" innovation_score="<<innovation_score<<" match_ms="<<match_ms
                   <<" candidate_posterior_dp="<<candidate_posterior_dp
                   <<" candidate_posterior_dr="<<candidate_posterior_dr
                   <<" sequence="<<state_sequence_<<" last_accepted="<<last_accepted_lidar_stamp_
                   <<" imu_degraded_total_s="<<(last_imu_?last_imu_->quality_.degraded_total_seconds:0.);
          if(geometry_quality_.valid && geometry_quality_.rank<6 && index_frame%options_.diagnostic_stride==0)
               LOG(INFO)<<"lio_weak_subspace stamp="<<std::setprecision(16)<<meas.lidar_end_time_
                        <<" l_ref_m="<<geometry_quality_.reference_length<<" normalized_basis=\n"
                        <<geometry_quality_.directions.leftCols(6-geometry_quality_.rank);
          if(accepted) {
               zjloc::common::Timer::Evaluate([&]() {
                    poseEstimation(p_frame,p_frame_aux_publish_only);
               }, "poseEstimate");
          }

          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
               } },
                                         "pub scalar data");

          index_frame++;
          p_frame->release();
          delete p_frame;
          if (p_frame_aux_publish_only != nullptr)
          {
               p_frame_aux_publish_only->release();
               delete p_frame_aux_publish_only;
          }
     }

     void lidarodom_m::poseEstimation(cloudFrame *p_frame, cloudFrame *p_frame_aux_publish_only)
     {
          // Called only after candidate acceptance and ESKF correction.
          // The auxiliary frame in non-fusion mode is output-only. buildFrame()
          // already performed motion compensation; refresh only its world points
          // with the optimized main-LiDAR pose before publishing. It is deliberately
          // not passed to optimize(), residual construction, or map insertion.
          if (p_frame_aux_publish_only != nullptr)
          {
               for (auto &point : p_frame_aux_publish_only->point_surf)
               {
                    transformPoint(point, current_state->rotation, current_state->translation);
               }
          }

          const Eigen::Vector3d current_translation = current_state->translation;
          const Eigen::Quaterniond current_rotation(current_state->rotation);
          bool should_update_map = true;
          if (index_frame > options_.init_num_frames && has_last_map_maintenance_pose_)
          {
               const double translation_delta = (current_translation - last_map_maintenance_translation_).norm();
               const double rotation_delta = (SO3(last_map_maintenance_rotation_).inverse()*SO3(current_rotation)).log().norm();
               const bool moved_enough =
                   translation_delta >= options_.map_update_translation_trigger ||
                   rotation_delta >= options_.map_update_rotation_trigger;
               const bool skipped_too_long =
                   options_.map_update_max_skip_frames > 0 &&
                   (index_frame - last_map_maintenance_frame_) >= options_.map_update_max_skip_frames;
               should_update_map = moved_enough || skipped_too_long;
          }

          if (should_update_map)
          { //   update map here
                zjloc::common::Timer::Evaluate([&]()
                                               { map_incremental(p_frame, p_frame_aux_publish_only, true); },
                                               "map update");
                has_last_map_maintenance_pose_ = true;
                last_map_maintenance_translation_ = current_translation;
                last_map_maintenance_rotation_ = current_rotation;
                last_map_maintenance_frame_ = index_frame;
          }
          else
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { map_incremental(p_frame, p_frame_aux_publish_only, false); },
                                              "map update");
          }

          const bool stride_hit = options_.fov_segment_stride <= 1 || (index_frame % options_.fov_segment_stride) == 0;
          if (should_update_map && (stride_hit || index_frame <= options_.init_num_frames))
          {
                zjloc::common::Timer::Evaluate([&]()
                                               { lasermap_fov_segment(); },
                                              "fov segment");
          }
     }

     bool lidarodom_m::optimize(cloudFrame *p_frame)
     {
          // Candidate is frame-local; persistent caches are committed only after gating.
          auto& candidate = *p_frame->p_state;
          Eigen::Quaterniond end_quat(candidate.rotation);
          Eigen::Vector3d end_t(candidate.translation);
          std::vector<point3D> keypoints;
          grid_sampling(p_frame->point_surf, keypoints, options_.sampling_rate * options_.surf_res);
          std::mt19937_64 rng(0); // reproducible selection, independent of previous rejected frames
          std::shuffle(keypoints.begin(), keypoints.end(), rng);
          ceres::Solver::Options solver_options;
          solver_options.max_num_iterations = 5;
          solver_options.num_threads = 3;
          solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
          for (int iter=0; iter<options_.max_num_iteration; ++iter) {
               for (auto& point : keypoints) point.point = end_quat * point.end_point + end_t;
               std::vector<ceres::CostFunction*> factors;
               final_correspondences_.clear();
               common::Timer::Evaluate([&] { addSurfCostFactor(factors, keypoints, p_frame); }, "factor_build_knn");
               match_quality_.residuals = static_cast<int>(factors.size());
               ceres::Problem problem;
               problem.AddParameterBlock(end_quat.coeffs().data(), 4, new ceres::EigenQuaternionManifold);
               problem.AddParameterBlock(end_t.data(), 3);
               auto* loss = factors.empty() ? nullptr : new ceres::HuberLoss(0.5);
               for (auto* factor : factors)
                    problem.AddResidualBlock(factor, loss, end_t.data(), end_quat.coeffs().data());
               if (match_quality_.residuals < options_.min_num_residuals) {match_quality_.reason="insufficient_support";return false;}
               const Eigen::Vector3d old_t=end_t;
               const Eigen::Quaterniond old_q=end_quat;
               ceres::Solver::Summary summary;
               common::Timer::Evaluate([&] { ceres::Solve(solver_options, &problem, &summary); }, "ceres_solve");
               match_quality_.initial_cost=summary.initial_cost;
               match_quality_.final_cost=summary.final_cost;
               if (!summary.IsSolutionUsable() || !end_t.allFinite() ||
                   !end_quat.coeffs().allFinite() || end_quat.norm()<1e-12) return false;
               end_quat.normalize();
               candidate.rotation=end_quat;
               candidate.translation=end_t;
               if ((end_t-old_t).norm()<options_.thres_translation_norm &&
                   AngularDistance(end_quat,old_q)<options_.thres_orientation_norm) break;
          }
          common::Timer::Evaluate([&] {
               geometry_quality_=EvaluateLidarGeometry(final_correspondences_,candidate.rotation.toRotationMatrix(),
                                                      candidate.translation,options_.geometry.reference_length);
          }, "geometry_diagnostic");
          match_quality_.usable=options_.max_num_iteration>0 && geometry_quality_.valid;
          return match_quality_.usable;
     }

     Neighborhood lidarodom_m::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     void lidarodom_m::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf,
                                         std::vector<point3D> &keypoints, const cloudFrame *p_frame)
     {

          auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                               Eigen::Vector3d &location, double &planarity_weight)
          {
               auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
               planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

               if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
               {
                    neighborhood.normal = -1.0 * neighborhood.normal;
               }
               return neighborhood;
          };

          double lambda_weight = std::abs(options_.weight_alpha);
          double lambda_neighborhood = std::abs(options_.weight_neighborhood);
          const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
          const double sum = lambda_weight + lambda_neighborhood;

          lambda_weight /= sum;
          lambda_neighborhood /= sum;

          const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
                                              ? 2
                                              : options_.voxel_neighborhood;

          const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
                                             ? 1
                                             : options_.threshold_voxel_occupancy;

          size_t num = keypoints.size();
          double knn_ms=0.;
          int num_residuals = 0;
          std::vector<LidarCorrespondence> candidates;
          const int candidate_budget=3*options_.max_num_residuals;
          candidates.reserve(candidate_budget);
          const double sparse_scene_distance_thresh_sq =
              options_.sparse_scene_distance_thresh * options_.sparse_scene_distance_thresh;

          for (int k = 0; k < num; k++)
          {
                auto &keypoint = keypoints[k];
                auto &raw_point = keypoint.raw_point;

                const bool sparse_far_point = raw_point.squaredNorm() >= sparse_scene_distance_thresh_sq;
                const int required_neighbors = sparse_far_point
                                                   ? std::max(options_.num_closest_neighbors + 2,
                                                              std::max(8, options_.min_number_neighbors / 2))
                                                   : options_.min_number_neighbors;
                const int max_neighbors = sparse_far_point
                                              ? std::max(required_neighbors, std::max(10, options_.max_number_neighbors / 2))
                                              : options_.max_number_neighbors;
                const double nearest_neighbor_limit =
                    options_.max_dist_to_plane_icp * options_.sparse_nearest_neighbor_ratio;
                const double max_neighbor_sq_distance = nearest_neighbor_limit * nearest_neighbor_limit;

                std::vector<voxel> voxels;
                const auto knn_start=std::chrono::steady_clock::now();
                auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
                                                        nb_voxels_visited,
                                                        options_.size_voxel_map,
                                                        max_neighbors,
                                                        kThresholdCapacity,
                                                        options_.estimate_normal_from_neighborhood
                                                            ? nullptr
                                                            : &voxels,
                                                        max_neighbor_sq_distance);
                knn_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-knn_start).count();

                if (vector_neighbors.size() < required_neighbors)
                     continue;

                double weight;

                Eigen::Vector3d location = keypoint.point;

                auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);

                if (neighborhood.a2D < options_.sparse_min_planarity)
                {
                     continue;
                }

                weight = lambda_weight * weight + lambda_neighborhood *
                                                      std::exp(-(vector_neighbors[0] -
                                                                 keypoint.point)
                                                                    .norm() /
                                                               (kMaxPointToPlane *
                                                                required_neighbors));

               double point_to_plane_dist;
               for (int i(0); i < options_.num_closest_neighbors; ++i)
               {
                    point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

                    if (point_to_plane_dist < options_.max_dist_to_plane_icp)
                    {

                         num_residuals++;

                         Eigen::Vector3d norm_vector = neighborhood.normal;
                         norm_vector.normalize();

                         candidates.push_back({keypoint.end_point,vector_neighbors[i],norm_vector,weight});

                         if (num_residuals >= candidate_budget) break;

                    }
               }

               if (num_residuals >= candidate_budget)
                    break;
          }
          common::Timer::Record("knn",knn_ms);
          const auto selected=SelectGeometricCoverage(candidates,options_.max_num_residuals,sampling_coverage_);
          for(size_t i:selected) {
               const auto& c=candidates[i];final_correspondences_.push_back(c);
               surf.push_back(CT_ICP::PointToPlaneFunctor::Create(c.anchor_world,c.end_imu,c.normal_world,c.weight));
          }
     }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom_m::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                  int nb_voxels_visited, double size_voxel_map,
                                  int max_num_neighbors, int threshold_voxel_capacity,
                                  std::vector<voxel> *voxels, double max_sq_distance)
     {

          if (max_num_neighbors <= 0)
          {
               if (voxels != nullptr)
                    voxels->clear();
               return {};
          }

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          // Collect all valid candidates first, then select KNN with nth_element.
          // This avoids per-candidate heap maintenance while preserving exact KNN semantics.
          thread_local std::vector<pair_distance_t> candidates;
          candidates.clear();
          const size_t min_candidate_capacity = static_cast<size_t>(max_num_neighbors) * 2;
          if (candidates.capacity() < min_candidate_capacity)
          {
               candidates.reserve(min_candidate_capacity);
          }

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search == map.end())
                         {
                              continue;
                         }

                         const auto &voxel_block = search.value();
                         if (voxel_block.NumPoints() < threshold_voxel_capacity)
                         {
                              continue;
                         }

                         for (int i(0); i < voxel_block.NumPoints(); ++i)
                         {
                              const auto &neighbor = voxel_block.points[i];
                              const double distance = (neighbor - point).squaredNorm();
                              if (max_sq_distance > 0.0 && distance > max_sq_distance)
                              {
                                   continue;
                              }
                              candidates.emplace_back(distance, neighbor, voxel_temp);
                         }
                    }
               }
          }

          if (candidates.empty())
          {
               if (voxels != nullptr)
                    voxels->clear();
               return {};
          }

          const size_t keep_size = std::min<size_t>(candidates.size(), static_cast<size_t>(max_num_neighbors));
          auto by_distance = [](const pair_distance_t &a, const pair_distance_t &b)
          {
               return std::get<0>(a) < std::get<0>(b);
          };

          if (candidates.size() > keep_size)
          {
               std::nth_element(candidates.begin(), candidates.begin() + keep_size, candidates.end(), by_distance);
               candidates.resize(keep_size);
          }
          std::sort(candidates.begin(), candidates.end(), by_distance);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors;
          closest_neighbors.reserve(keep_size);
          if (voxels != nullptr)
          {
               voxels->clear();
               voxels->reserve(keep_size);
          }

          for (const auto &candidate : candidates)
          {
               closest_neighbors.push_back(std::get<1>(candidate));
               if (voxels != nullptr)
                    voxels->push_back(std::get<2>(candidate));
          }

          return closest_neighbors;
     }

     void lidarodom_m::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                     double voxel_size,
                                     int max_num_points_in_voxel, double min_distance_points,
                                     int min_num_points)
     {
          short kx = static_cast<short>(point[0] / voxel_size);
          short ky = static_cast<short>(point[1] / voxel_size);
          short kz = static_cast<short>(point[2] / voxel_size);

          voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

          if (search != map.end())
          {
               auto &voxel_block = (search.value());

               if (!voxel_block.IsFull())
               {
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block.NumPoints(); ++i)
                    {
                         auto &_point = voxel_block.points[i];
                         double sq_dist = (_point - point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points)
                         {
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                    {
                         if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                         {
                              voxel_block.AddPoint(point);
                         }
                    }
               }
          }
          else
          {
               if (min_num_points <= 0)
               {
                    voxelBlock block(max_num_points_in_voxel);
                    block.AddPoint(point);
                    map[voxel(kx, ky, kz)] = std::move(block);
               }
          }
     }

     void lidarodom_m::map_incremental(cloudFrame *p_frame, cloudFrame *p_frame_aux_publish_only,
                                       bool update_map, int min_num_points)
     {
          //   only surf
          const size_t aux_point_count = p_frame_aux_publish_only == nullptr
                                             ? 0
                                             : p_frame_aux_publish_only->point_surf.size();
          points_world->points.reserve(p_frame->point_surf.size() + aux_point_count);
          for (auto &point : p_frame->point_surf)
          {
               if (update_map)
               {
                    addPointToMap(voxel_map, point.point,
                                  options_.size_voxel_map, options_.max_num_points_in_voxel,
                                  options_.min_distance_points, min_num_points);

               }
               auto &p = points_world->points.emplace_back();
               p.x = point.point.x();
               p.y = point.point.y();
               p.z = point.point.z();
               p.intensity = point.intensity;
          }
          common::Timer::Evaluate([&] {
               publishFrameProducts(SafeSE3(current_state->rotation, current_state->translation),
                                    p_frame->time_frame_end, p_frame_aux_publish_only);
          }, "cloud_products");
          points_world->clear();
     }

     void lidarodom_m::publishFrameProducts(const SE3 &pose_of_lo, double stamp,
                                            const cloudFrame *p_frame_aux_publish_only)
     {
          auto filter_output_cloud = [&]()
          {
               pcl::PassThrough<pcl::PointXYZI> pass;
               pass.setInputCloud(points_world);
               pass.setFilterFieldName("z");
               pass.setFilterLimits(cloud_pub_options.min_z_filter, cloud_pub_options.max_z_filter);
               pass.filter(*points_world);

               if (cloud_pub_options.enable_body_filter)
               {
                    const Eigen::Matrix3d body_to_world_rot = pose_of_lo.rotationMatrix();
                    const Eigen::Matrix3f world_to_body_rot = body_to_world_rot.transpose().cast<float>();
                    const Eigen::Vector3f world_to_body_trans =
                        (-body_to_world_rot.transpose() * pose_of_lo.translation()).cast<float>();

                    Eigen::Affine3f world_to_body = Eigen::Affine3f::Identity();
                    world_to_body.linear() = world_to_body_rot;
                    world_to_body.translation() = world_to_body_trans;

                    pcl::CropBox<pcl::PointXYZI> crop;
                    crop.setInputCloud(points_world);
                    crop.setTransform(world_to_body);
                    crop.setMin(Eigen::Vector4f(cloud_pub_options.body_filter_x_min,
                                                cloud_pub_options.body_filter_y_min,
                                                cloud_pub_options.body_filter_z_min,
                                                1.0f));
                    crop.setMax(Eigen::Vector4f(cloud_pub_options.body_filter_x_max,
                                                cloud_pub_options.body_filter_y_max,
                                                cloud_pub_options.body_filter_z_max,
                                                1.0f));
                    crop.setNegative(true);
                    crop.filter(*points_world);
               }
          };

          // Keep ScanContext on the main LIO cloud in non-fusion mode. The
          // auxiliary frame is appended only after this callback, so disabling
          // bundle fusion means exactly "publish only" for the auxiliary points.
          bool main_cloud_already_filtered = false;
          if (pub_scantext_data)
          {
               filter_output_cloud();
               main_cloud_already_filtered = true;
               pub_scantext_data(points_world, pose_of_lo, stamp);
          }

          const bool has_aux_publish_points =
              p_frame_aux_publish_only != nullptr && !p_frame_aux_publish_only->point_surf.empty();
          if (has_aux_publish_points)
          {
               for (const auto &point : p_frame_aux_publish_only->point_surf)
               {
                    auto &p = points_world->points.emplace_back();
                    p.x = point.point.x();
                    p.y = point.point.y();
                    p.z = point.point.z();
                    p.intensity = point.intensity;
               }
          }

          if (!main_cloud_already_filtered || has_aux_publish_points)
          {
               // If ScanContext already filtered the main cloud, applying these
               // filters again handles only the newly appended auxiliary points.
               filter_output_cloud();
          }

          CloudPtr down(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::VoxelGrid<pcl::PointXYZI> vg;
          vg.setInputCloud(points_world);
          vg.setLeafSize(cloud_pub_options.space_down_sample,
                         cloud_pub_options.space_down_sample,
                         cloud_pub_options.space_down_sample);
          vg.filter(*down);

          if (pub_cloud_to_ros)
          {
               std::string laser_topic = "laser";
               pub_cloud_to_ros(laser_topic, down, stamp);
          }

     }

     void lidarodom_m::lasermap_fov_segment()
     {
          //   use predict pose here
          Eigen::Vector3d location = current_state->translation;
          const double max_distance_sq = options_.max_distance * options_.max_distance;
          std::vector<voxel> voxels_to_erase;
          voxels_to_erase.reserve(voxel_map.size() / 8 + 1);
          for (const auto &pair : voxel_map)
          {
               if (pair.second.points.empty())
               {
                    voxels_to_erase.push_back(pair.first);
                    continue;
               }

               const Eigen::Vector3d &pt = pair.second.points.front();
               if ((pt - location).squaredNorm() > max_distance_sq)
               {
                    voxels_to_erase.push_back(pair.first);
               }
          }

          for (const auto &vox : voxels_to_erase)
          {
               voxel_map.erase(vox);
          }

     }

     cloudFrame *lidarodom_m::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                         double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf = std::move(const_surf);
          Undistort(frame_surf);
          for (auto& point : frame_surf)
               transformPoint(point, cur_state->rotation, cur_state->translation);

          std::vector<point3D> empty_const_surf;
          cloudFrame *p_frame = new cloudFrame(std::move(frame_surf), std::move(empty_const_surf), cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          return p_frame;
     }

     void lidarodom_m::stateInitialization()
     {
          const auto& begin=imu_states_.front();
          current_state->rotation_begin=begin.R_.unit_quaternion();
          current_state->translation_begin=begin.p_;
          current_state->velocity_begin=begin.v_;
          current_state->ba_begin=begin.ba_;
          current_state->bg_begin=begin.bg_;
          CommitFrameState(eskf_.GetNominalState(),*current_state);
     }

     std::vector<MeasureGroup> lidarodom_m::getMeasureMents()
     {
          std::vector<MeasureGroup> measurements;
          // std::cout << "get measurements called." << std::endl;
          while (true)
          {
               if (imu_buffer_.empty())
                    return measurements;

               if (lidar_buffer_.empty())
                    return measurements;

               const double lidar_begin_time = time_buffer_.front().first;
               const double lidar_end_time = lidar_begin_time + time_buffer_.front().second;

               if (imu_buffer_.back()->timestamp_ < lidar_end_time + delay_time_)
                    return measurements;

               MeasureGroup meas;

               meas.lidar_ = std::move(lidar_buffer_.front());
               meas.ingress_time_ = ingress_time_buffer_.front();
               meas.lidar_begin_time_ = time_buffer_.front().first;
               meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
               lidar_buffer_.pop_front();
               ingress_time_buffer_.pop_front();
               time_buffer_.pop_front();

               // std::cout << "end lidar" << std::endl;
               meas.imu_.clear();
               while(!imu_buffer_.empty() && imu_buffer_.front()->timestamp_<=meas.lidar_end_time_) {
                    meas.imu_.push_back(imu_buffer_.front());imu_buffer_.pop_front();
               }
               {
                    const double crop_begin = meas.lidar_begin_time_;
                    const double crop_end = meas.lidar_end_time_;
                    const double sync_margin = std::max(0.0, options_.aux_lidar_sync_margin);
                    const double query_begin = crop_begin - sync_margin;
                    const double query_end = crop_end + sync_margin;
                    const double aux_offset = options_.aux_lidar_time_offset;

                    meas.aux_lidar_.clear();
                    meas.aux_lidar_.reserve(13000);

                    while (!aux_lidar_buffer_.empty())
                    {
                         auto &aux_time = aux_lidar_time_buffer_.front();
                         auto &aux = aux_lidar_buffer_.front();

                         if (aux.empty())
                         {
                              aux_lidar_buffer_.pop_front();
                              aux_lidar_time_buffer_.pop_front();
                              aux_point_cursor_ = 0;
                              continue;
                         }

                         const double start_time = aux_time.first + aux_offset;
                         const double end_time = aux_time.first + aux_time.second + aux_offset;

                         // 1. aux 整帧在查询窗口之前 → 丢弃整帧
                         if (end_time < query_begin)
                         {
                              aux_lidar_buffer_.pop_front();
                              aux_lidar_time_buffer_.pop_front();
                              aux_point_cursor_ = 0;
                              continue;
                         }

                         // 2. aux 整帧在查询窗口之后 → 等下次主雷达帧
                         if (start_time > query_end)
                         {
                              break;
                         }

                         CropAuxPoints(aux,aux_point_cursor_,crop_begin,crop_end,aux_offset,meas.aux_lidar_);

                         // 4. 当前 aux 帧已经消费完，继续尝试下一帧，避免一个主帧内
                         //    多个 aux 包被后一个覆盖。
                         if (aux_point_cursor_ >= aux.size())
                         {
                              aux_lidar_buffer_.pop_front();
                              aux_lidar_time_buffer_.pop_front();
                              aux_point_cursor_ = 0;
                              continue;
                         }

                         // 当前 aux 帧还有未来点，后面的 aux 帧时间只会更晚，等待下次。
                         break;
                    }

                    if (meas.aux_lidar_.size() < min_aux_points_)
                    {
                         meas.aux_lidar_.clear();
                    }
                    else
                    {
                         meas.aux_lidar_time_ = crop_begin;
                    }

                    if(!imu_buffer_.empty())meas.imu_.push_back(imu_buffer_.front()); // read-only right bracket; not consumed
               }

                measurements.emplace_back(std::move(meas));
                return measurements; // bound queue lock: one scan per transaction
          }
     }

     bool lidarodom_m::Predict()
     {
          if(prediction_fault_ || !last_imu_)return false;
          imu_states_.push_back(eskf_.GetNominalState());
          const double end=measures_.lidar_end_time_;
          for(const auto& imu:measures_.imu_) {
               if(!imu)return false;
               const double current=eskf_.GetNominalState().timestamp_;
               if(imu->timestamp_<=current)continue;
               const double to=std::min(end,imu->timestamp_);
               if(to<=current)break;
               if(!PropagateInterval(eskf_,*last_imu_,*imu,to))return false;
               imu_states_.push_back(eskf_.GetNominalState());
               if(to==imu->timestamp_)last_imu_=imu;
               // If to lies inside [last_imu_,imu], retain the RAW left endpoint;
               // next frame uses the same interpolation convention as replay.
               if(to==end)break;
          }
          return imu_states_.size()>=2 && std::abs(eskf_.GetNominalState().timestamp_-end)<1e-9;
     }

     void lidarodom_m::Undistort(std::vector<point3D> &points)
     {
          const size_t before=points.size();
          const SE3 end=eskf_.GetNominalSE3();
          points.erase(std::remove_if(points.begin(),points.end(),[&](point3D& p) {
               return !DeskewToEndImu(p.raw_point,p.timestamp,imu_states_,TIL_,end,p.end_point);
          }),points.end());
          if (before!=points.size()) LOG(WARNING)<<"deskew_uncovered_or_invalid="<<(before-points.size())
              <<" stamp="<<std::setprecision(16)<<measures_.lidar_end_time_;
     }

     void lidarodom_m::TryInitIMU()
     {
          static double last_init_stamp=-std::numeric_limits<double>::infinity();
          for (auto imu : measures_.imu_)
          {
               if(imu->timestamp_<=last_init_stamp || imu->quality_.degraded || imu->quality_.held_mask)continue;
               last_init_stamp=imu->timestamp_;
               imu_init_.AddIMU(*imu);
          }

          if (imu_init_.InitSuccess())
          {
               // 读取初始零偏，设置ESKF
               zjloc::ESKFD::Options options;
               // 噪声由初始化器估计
               // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
               // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
               // options.update_bias_acce_ = false;
               // options.update_bias_gyro_ = false;
               eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
               if (!measures_.imu_.empty())
               {
                    const IMUPtr &last_init_imu = measures_.imu_.back();
                    eskf_.SetInitialTime(last_init_imu->timestamp_);
                    last_imu_ = last_init_imu;
               }
               else
               {
                    eskf_.SetInitialTime(measures_.lidar_end_time_);
                    last_imu_ = nullptr;
               }
               gyro_measurement_variance_=imu_init_.GetCovGyro().cwiseMax(1e-12);
               last_accepted_lidar_stamp_=eskf_.GetNominalState().timestamp_;
               imu_need_init_ = false;

               std::cout << ANSI_COLOR_GREEN_BOLD << "IMU初始化成功" << ANSI_COLOR_RESET << std::endl;
          }
     }

}
