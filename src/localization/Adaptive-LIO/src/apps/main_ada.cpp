// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <deque>
#include <algorithm>

// ros2 lib
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <pcl/common/transforms.h>

#include "common/utility.h"
#include "algo/imu_startup.h"
#include "preprocess/cloud_convert/cloud_convert2.h"
#include "lio/lidarodom.h"

#include "scantext_module/Mapping.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

nav_msgs::msg::Path laserOdoPath;

DEFINE_string(config_file, "", "Path to adaptive_lio config YAML.");

zjloc::lidarodom_m *lio;
zjloc::CloudConvert2 *convert;
std::shared_ptr<scantext::MappingCore> scantext_mapping;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
std::shared_ptr<class ImuOdomOutput> imu_odom_output;
double gnorm = 1.0;
zjloc::ImuQualityGate imu_quality_gate;

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))

inline rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}

#include "apps/imu_odom_output.h"

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    const auto ingress_time = std::chrono::steady_clock::now();
    // cloud_vec	一整帧 Livox 点云，被切成的多个子点云
    // cloud_out	第 i 个子点云（时间片）
    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;

    // Use the ROS shared pointer directly to avoid copying the full Livox message.
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out, false); },
                                   "laser convert");

    if (cloud_vec.empty() || t_out.empty())
    {
        return;
    }

    const size_t segment_count = std::min(cloud_vec.size(), t_out.size());
    for (size_t i = 0; i < segment_count; i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = 0.01;
        thread_local std::mt19937_64 g;
        zjloc::common::Timer::Evaluate([&]()
                                       { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size); },
                                       "laser ds");
        lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false, ingress_time);
        // pair<本段数据的绝对起始时间,数据持续时长>
    }
}

void aux_livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    std::vector<std::vector<point3D>> cloud_vec; // 附属雷达只存于第0个时间片
    std::vector<double> t_out;                   // 只有第0个时间片，代表总体的时间长度

    // Keep auxiliary LiDAR behavior unchanged; only remove the full-message copy.
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out, true); },
                                   "laser convert");

    if (cloud_vec.empty() || t_out.empty())
    {
        return;
    }

    auto &cloud_out = cloud_vec[0];
    double sample_size = 0.01;
    thread_local std::mt19937_64 g;
    zjloc::common::Timer::Evaluate([&]()
                                   { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        sub_sample_frame(cloud_out, sample_size);
        // Sort/finite validation is centralized in pushData before queue locking.
         },
                                   "laser ds");

    lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, t_out[0]), true);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const auto ingress_time = std::chrono::steady_clock::now();
    // Gazebo 原始快照为 100 Hz；解码前按采样时间取 20 Hz，避免挤占 IMU 和导航线程。
    static double last_snapshot_stamp = -std::numeric_limits<double>::infinity();
    const double snapshot_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (snapshot_stamp > last_snapshot_stamp && snapshot_stamp - last_snapshot_stamp < 0.05 - 1e-9)
    {
        return;
    }
    last_snapshot_stamp = snapshot_stamp;
    // sensor_msgs::msg::PointCloud2::SharedPtr cloud(new sensor_msgs::msg::PointCloud2(*msg));

    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out); },
                                   "laser convert");

    const size_t segment_count = std::min(cloud_vec.size(), t_out.size());
    for (size_t i = 0; i < segment_count; i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;
        // double sample_size = 0.05;
        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            thread_local std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size);
        },
                                       "laser ds");

        // 在ROS2中使用的是nanoseconds().count()来获取时间戳
        lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false, ingress_time);
    }
}

void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    // sensor_msgs::msg::Imu::SharedPtr msg_temp(new sensor_msgs::msg::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * gnorm);
    static zjloc::ImuStartupGate startup_gate;
    if (!startup_gate.Ready(*imu)) {
        LOG_EVERY_N(INFO, 200) << "Waiting for one second of continuous IMU timestamps before initialization";
        return;
    }
    // Once, after SI conversion: share processed values AND immutable quality with both chains.
    const auto decision=imu_quality_gate.Process(*imu);
    if(decision==zjloc::ImuQualityGate::Decision::Fault) {
        lio->Fail();
        LOG_FIRST_N(ERROR,1)<<std::setprecision(18)
            <<"IMU quality/time fault; stopped both chains, restart node. reason="<<imu->quality_.reasons
            <<" stamp="<<imu->timestamp_<<" previous="<<imu_quality_gate.LastStamp()
            <<" dt="<<imu->timestamp_-imu_quality_gate.LastStamp();
        return;
    }
    if(decision==zjloc::ImuQualityGate::Decision::Drop) {
        LOG_EVERY_N(WARNING,200)<<"drop invalid/stale IMU reason="<<imu->quality_.reasons;return;
    }
    if(imu->quality_.degraded) { LOG_EVERY_N(WARNING,20)<<"IMU degraded held_mask="<<int(imu->quality_.held_mask)
        <<" saturation_mask="<<int(imu->quality_.saturation_mask)<<" accumulated_s="<<imu->quality_.degraded_total_seconds; }
    if (imu_odom_output)
    {
        imu_odom_output->OnImu(imu);
    }
    lio->pushData(imu);
}

int main(int argc, char **argv)
{
    // 处理命令行参数，将gflags参数与ROS2参数分离
    // Debug: print initial argv for diagnosing flag parsing issues
    std::cout << "[adaptive_lio] argv before init:";
    for (int i = 0; i < argc; ++i)
    {
        std::cout << " '" << (argv[i] ? argv[i] : std::string("(null)")) << "'";
    }
    std::cout << std::endl;

    // Initialize logging and ROS2. Important: call rclcpp::init before
    // parsing Google gflags so ROS2's --ros-args (and -r) are handled by
    // rclcpp and not consumed (or rejected) by gflags.
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    rclcpp::init(argc, argv);

    // Build a filtered argv for gflags so ROS2 launch args are not seen by gflags.
    // Keep program name.
    std::vector<char *> gflags_argv;
    gflags_argv.push_back(argv[0]);
    for (int i = 1; i < argc; ++i)
    {
        std::string a(argv[i]);
        // Skip ROS2-specific markers and remappings
        if (a == "--ros-args")
        {
            continue;
        }
        if (a.rfind("--ros", 0) == 0)
        {
            continue;
        }
        if (a == "-r")
        {
            // remap flag, skip it (its argument is the remapping spec which
            // usually starts with __node:= or topic remap; skip that token too)
            continue;
        }
        if (a.rfind("__", 0) == 0)
        {
            // __node:=... or other remap specs
            continue;
        }
        if (a == "--params-file")
        {
            // skip params-file and its value (if present)
            ++i;
            continue;
        }
        // otherwise keep the arg for gflags
        gflags_argv.push_back(argv[i]);
    }

    int gflags_argc = static_cast<int>(gflags_argv.size());
    char **gflags_argv_ptr = gflags_argv.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);
    auto node = rclcpp::Node::make_shared("adaptive_lio_node");

    std::string config_file = FLAGS_config_file.empty()
                                  ? std::string(ROOT_DIR) + "config/mapping_m.yaml"
                                  : FLAGS_config_file;
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    // Init Scantext Modules
    scantext_mapping = std::make_shared<scantext::MappingCore>();
    scantext::SCParams scantext_params;

    // Load configs
    try
    {
        auto yaml_mapping_ = YAML::LoadFile(config_file);
        if (yaml_mapping_["mapping_module"])
        {
            scantext::MappingCore::Config cfg;
            auto node = yaml_mapping_["mapping_module"];
            if (node["enable_mapping"])
                cfg.enable_mapping = node["enable_mapping"].as<bool>();
            if (node["keyframe_dist_thresh"])
                cfg.keyframe_dist_thresh = node["keyframe_dist_thresh"].as<double>();
            if (node["keyframe_angle_thresh"])
                cfg.keyframe_angle_thresh = node["keyframe_angle_thresh"].as<double>();
            if (node["map_save_path"])
                cfg.map_save_path = node["map_save_path"].as<std::string>();
            if (node["auto_save_interval"])
                cfg.auto_save_interval = node["auto_save_interval"].as<double>();
            scantext_mapping->setConfig(cfg);

            if (node["scantext"])
            {
                auto sc_node = node["scantext"];
                if (sc_node["num_ring"])
                    scantext_params.num_ring = sc_node["num_ring"].as<int>();
                if (sc_node["num_sector"])
                    scantext_params.num_sector = sc_node["num_sector"].as<int>();
                if (sc_node["max_radius"])
                    scantext_params.max_radius = sc_node["max_radius"].as<double>();
                if (sc_node["lidar_height"])
                    scantext_params.lidar_height = sc_node["lidar_height"].as<double>();
                if (sc_node["use_scpp"])
                    scantext_params.use_scpp = sc_node["use_scpp"].as<bool>();
                if (sc_node["scpp_search_ratio"])
                    scantext_params.scpp_search_ratio = sc_node["scpp_search_ratio"].as<double>();
                if (sc_node["cart_x_unit"])
                    scantext_params.cart_x_unit = sc_node["cart_x_unit"].as<double>();
                if (sc_node["cart_y_unit"])
                    scantext_params.cart_y_unit = sc_node["cart_y_unit"].as<double>();
                if (sc_node["cart_x_max"])
                    scantext_params.cart_x_max = sc_node["cart_x_max"].as<double>();
                if (sc_node["cart_y_max"])
                    scantext_params.cart_y_max = sc_node["cart_y_max"].as<double>();
            }
            scantext_mapping->setScanContextParams(scantext_params);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading scantext configs: " << e.what() << std::endl;
    }

    lio = new zjloc::lidarodom_m();
    if (!lio->init(config_file))
    {
        return -1;
    }

    auto yaml_cfg = YAML::LoadFile(config_file);

    auto pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/scan", rclcpp::SensorDataQoS().keep_last(1));

    struct ScanPublishTask
    {
        zjloc::CloudPtr cloud;
        double time;
    };

    std::deque<ScanPublishTask> scan_publish_queue;
    std::mutex scan_publish_queue_mutex;
    std::condition_variable scan_publish_queue_cv;
    bool stop_scan_publish_thread = false;
    std::thread scan_publish_worker([&]()
                                    {
        while (true)
        {
            ScanPublishTask task;
            {
                std::unique_lock<std::mutex> lock(scan_publish_queue_mutex);
                scan_publish_queue_cv.wait(lock, [&]() {
                    return stop_scan_publish_thread || !scan_publish_queue.empty();
                });

                if (stop_scan_publish_thread && scan_publish_queue.empty())
                {
                    break;
                }

                task = std::move(scan_publish_queue.front());
                scan_publish_queue.pop_front();
            }

            if (!task.cloud)
            {
                continue;
            }

            sensor_msgs::msg::PointCloud2 cloud_output;
            pcl::toROSMsg(*task.cloud, cloud_output);
            cloud_output.header.stamp = get_ros_time(task.time);
            cloud_output.header.frame_id = "odom";
            pub_scan->publish(cloud_output);
        } });

    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            if (topic_name != "laser" || pub_scan->get_subscription_count() == 0)
            {
                return true;
            }

            {
                std::lock_guard<std::mutex> lock(scan_publish_queue_mutex);
                if (scan_publish_queue.size() >= 2)
                {
                    scan_publish_queue.pop_front();
                }
                scan_publish_queue.push_back({cloud, time});
            }
            scan_publish_queue_cv.notify_one();
            return true;
        }

    );

    auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
    auto pubLaserOdometryPath = node->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);

    // 创建tf广播器
    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    imu_odom_output = std::make_shared<ImuOdomOutput>(
        laserOdoPath,
        pubLaserOdometry,
        pubLaserOdometryPath,
        node->get_clock(),
        g_tf_broadcaster.get(),
        "odom",
        "aft_mapped");

    // User setup: IMU is colocated/aligned with aft_mapped frame
    imu_odom_output->SetAftToBaseTransform(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    imu_odom_output->SetBaseLinkFrame("base_link");

    if (yaml_cfg["common"])
    {
        const auto common = yaml_cfg["common"];
        if (common["fusion_base_link_frame"])
        {
            imu_odom_output->SetBaseLinkFrame(common["fusion_base_link_frame"].as<std::string>());
        }
        if (common["fusion_aft_to_base_xyzrpy"])
        {
            auto arr = common["fusion_aft_to_base_xyzrpy"].as<std::vector<double>>();
            if (arr.size() == 6)
            {
                imu_odom_output->SetAftToBaseTransform(
                    Eigen::Vector3d(arr[0], arr[1], arr[2]),
                    Eigen::Vector3d(arr[3], arr[4], arr[5]));
            }
        }
        if (common["fusion_max_path_poses"])
        {
            imu_odom_output->SetMaxPathPoses(common["fusion_max_path_poses"].as<size_t>());
        }
    }

    lio->SetFaultCallback([] { imu_odom_output->Fail(); });
    imu_odom_output->SetFaultCallback([] { lio->RequestStop(); });
    imu_odom_output->Start();
    lio->SetStateCallback([](const zjloc::LioStateUpdate& state) {
        imu_odom_output->OnState(state);
    });
    // Scantext processing queue
    struct ScantextTask
    {
        scantext::ScanContext::SCDescriptor sc;
        scantext::ScanContext::RingKey rk;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds;
        Eigen::Isometry3d odom_pose;
        double time;
    };

    const bool scantext_enabled = scantext_mapping && scantext_mapping->getConfig().enable_mapping;

    std::queue<ScantextTask> scantext_queue;
    std::mutex scantext_queue_mutex;
    std::condition_variable scantext_queue_cv;
    bool stop_scantext_thread = false;
    std::thread scantext_worker;

    // Consumer thread function
    if (scantext_enabled)
    {
        scantext_worker = std::thread([&]()
                                      {
    while (!stop_scantext_thread) {
        ScantextTask task;
        {
            std::unique_lock<std::mutex> lock(scantext_queue_mutex);
            // Wait for task or stop signal
            scantext_queue_cv.wait(lock, [&]() { return !scantext_queue.empty() || stop_scantext_thread; });

            if (stop_scantext_thread && scantext_queue.empty()) break;

            task = scantext_queue.front();
            scantext_queue.pop();
        }

        if (!scantext_mapping) continue;
        

        // 1) mapping：如果 enable_mapping，且满足内部 keyframe 条件，则入库
        std::shared_ptr<scantext::KeyFrame> new_kf;
        if (scantext_mapping->getConfig().enable_mapping) {
            bool added = scantext_mapping->addFrameWithSC(
                task.cloud_ds, task.odom_pose, task.time, task.sc, task.rk, &new_kf);

            // if (added && new_kf) {
            //     // 增量更新 relo 的 map（重要！否则 mapping 模式下 relo 永远空 map）
            //     scantext_relo->addKeyFrame(new_kf);
            // }
        }
    } });
    }

    Eigen::Isometry3d last_sc_pose = Eigen::Isometry3d::Identity();
    double last_sc_time = 0.0;
    bool has_last_sc = false;

    scantext::ScanContext sc_extractor(scantext_params);

    auto scantext_cbk =
        std::function<bool(const zjloc::CloudPtr &, const SE3 &, double)>(
            [&](const zjloc::CloudPtr &points_world,
                const SE3 &pose,
                double time) -> bool
            {
                if (!scantext_mapping || !points_world || !scantext_mapping->getConfig().enable_mapping)
                    return true;

                // --- pose -> Eigen ---
                Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
                eigen_pose.linear() = pose.rotationMatrix();
                eigen_pose.translation() = pose.translation();

                // --- Step2: 只在关键帧/低频做 SC ---
                const double dist_th =
                    scantext_mapping->getConfig().keyframe_dist_thresh > 1e-6 ? scantext_mapping->getConfig().keyframe_dist_thresh : 1.0;

                const double ang_th =
                    scantext_mapping->getConfig().keyframe_angle_thresh > 1e-6 ? scantext_mapping->getConfig().keyframe_angle_thresh : 0.2;

                const double time_th = 1.0; // 1Hz 兜底（可放到 yaml）

                bool do_sc = false;
                if (!has_last_sc)
                {
                    do_sc = true;
                }
                else
                {
                    Eigen::Isometry3d delta = last_sc_pose.inverse() * eigen_pose;
                    double dist = delta.translation().norm();
                    double ang = Eigen::AngleAxisd(delta.rotation()).angle();
                    double dt = time - last_sc_time;

                    do_sc = (dist >= dist_th) || (ang >= ang_th) || (dt >= time_th);
                }

                if (!do_sc)
                {
                    return true; // 不做 SC，直接返回，不占用队列
                }

                has_last_sc = true;
                last_sc_pose = eigen_pose;
                last_sc_time = time;

                auto cloud_local = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                if (!points_world->empty())
                {
                    const Eigen::Matrix4f t_imu_world = eigen_pose.matrix().cast<float>().inverse();
                    pcl::transformPointCloud(*points_world, *cloud_local, t_imu_world);
                }

                // Descriptor uses committed frame-end IMU coordinates; pose is T_WI.
                auto sc = sc_extractor.makeScanContext(*cloud_local);
                auto rk = sc_extractor.makeRingKey(sc);

                // --- 为 ICP / (可选)建图存储 构造一个小云（上限 2000 点）---
                constexpr size_t MAX_DS_PTS = 2000;
                auto cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                if (!cloud_local->empty())
                {
                    cloud_ds->reserve(std::min(cloud_local->size(), MAX_DS_PTS));
                    size_t step = std::max<size_t>(1, cloud_local->size() / MAX_DS_PTS);

                    for (size_t i = 0; i < cloud_local->size(); i += step)
                    {
                        cloud_ds->push_back(cloud_local->points[i]);
                    }
                }

                // --- push 轻量任务到队列 ---
                {
                    std::lock_guard<std::mutex> lock(scantext_queue_mutex);
                    if (scantext_queue.size() < 5)
                    {
                        scantext_queue.push({sc, rk, cloud_ds, eigen_pose, time});
                    }
                }
                scantext_queue_cv.notify_one();

                return true;
            });

    auto vel_pub = node->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
    auto dist_pub = node->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);

    auto save_map_srv =
        node->create_service<std_srvs::srv::Trigger>(
            "save_map_service",
            [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
            {
                std::string path = scantext_mapping->getConfig().map_save_path;
                std::string db_path = std::string(ROOT_DIR) + "map_db";

                scantext_mapping->saveMapAsync(path);

                res->success = true;
                res->message = "Map saving started asynchronously";
            });

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::msg::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub->publish(time_rviz);
            else
                dist_pub->publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(data_pub_func);
    if (scantext_enabled)
    {
        lio->setFunc(scantext_cbk);
    }

    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);

    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    std::string laser_topic = yaml_cfg["common"]["lid_topic"].as<std::string>();
    const auto aux_topic_cfg = yaml_cfg["common"]["aux_lidar_topic"];
    const std::string aux_laser_topic = aux_topic_cfg ? aux_topic_cfg.as<std::string>() : "";
    std::string imu_topic = yaml_cfg["common"]["imu_topic"].as<std::string>();
    gnorm = yaml_cfg["common"]["gnorm"].as<double>();
    if(!std::isfinite(gnorm) || gnorm<=0)throw std::runtime_error("gnorm must be finite and positive");
    zjloc::ImuQualityOptions imu_quality_options;
    const auto iq=yaml_cfg["imu_quality"];
    if(iq) {
        auto vector_option=[&](const char* key,Vec3d& out){
            if(!iq[key])return;
            const auto values=iq[key].as<std::vector<double>>();
            if(values.size()!=3)throw std::runtime_error(std::string(key)+" must have three SI values");
            out=Vec3d(values[0],values[1],values[2]);
        };
        vector_option("gyro_limit_rad_s",imu_quality_options.gyro_limit);
        vector_option("accel_limit_m_s2",imu_quality_options.accel_limit);
        if(iq["saturation_ratio"])imu_quality_options.saturation_ratio=iq["saturation_ratio"].as<double>();
        if(iq["hold_seconds"])imu_quality_options.hold_seconds=iq["hold_seconds"].as<double>();
        if(iq["noise_multiplier"])imu_quality_options.noise_multiplier=iq["noise_multiplier"].as<double>();
        if(iq["recovery_samples"])imu_quality_options.recovery_samples=iq["recovery_samples"].as<int>();
    }
    imu_quality_gate=zjloc::ImuQualityGate(imu_quality_options);
    if(imu_quality_options.gyro_limit.isZero() || imu_quality_options.accel_limit.isZero())
        LOG(WARNING)<<"IMU saturation limits unspecified for at least one sensor; configure actual SI ranges";


    auto sensor_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_sub_options;
    sensor_sub_options.callback_group = sensor_callback_group;

    // 传感器订阅兼容仿真 Best Effort 与实车 Reliable 发布端。
    const auto lidar_qos = rclcpp::SensorDataQoS().keep_last(1);
    std::shared_ptr<void> subLaserCloud =
        (convert->lidar_type_ == zjloc::CloudConvert2::LidarType::AVIA)
            ? std::static_pointer_cast<void>(node->create_subscription<livox_ros_driver2::msg::CustomMsg>(laser_topic, lidar_qos, livox_pcl_cbk, sensor_sub_options))
            : std::static_pointer_cast<void>(node->create_subscription<sensor_msgs::msg::PointCloud2>(laser_topic, lidar_qos, standard_pcl_cbk, sensor_sub_options));

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subAuxLaserCloud;
    if (!aux_laser_topic.empty()) {
        subAuxLaserCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(aux_laser_topic, lidar_qos, aux_livox_pcl_cbk, sensor_sub_options);
    }

    auto imu_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_sub_options;
    imu_sub_options.callback_group = imu_callback_group;
    auto sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 500, imuHandler, imu_sub_options);

    std::thread measurement_process([] {
        try { lio->run(); }
        catch(const std::exception& e) { LOG(ERROR)<<"LIO worker failed: "<<e.what(); lio->Fail(); }
    });

    const size_t executor_threads = std::max<size_t>(2, std::min<size_t>(4, std::thread::hardware_concurrency()));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
    executor.add_node(node);
    executor.spin();

    // Stop producers and join them before stopping cloud consumers/publishers.
    lio->RequestStop();
    if (measurement_process.joinable()) measurement_process.join();
    imu_odom_output->Stop();

    // Cleanup Scantext thread
    {
        std::lock_guard<std::mutex> lock(scantext_queue_mutex);
        stop_scantext_thread = true;
    }
    scantext_queue_cv.notify_all();
    if (scantext_worker.joinable())
        scantext_worker.join();

    {
        std::lock_guard<std::mutex> lock(scan_publish_queue_mutex);
        stop_scan_publish_thread = true;
    }
    scan_publish_queue_cv.notify_all();
    if (scan_publish_worker.joinable())
        scan_publish_worker.join();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    delete lio;
    delete convert;
    return 0;
}
