#include "cloud_convert2.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
// #include <execution>

namespace zjloc
{

    void CloudConvert2::Process(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg,
        std::vector<std::vector<point3D>> &pcl_out,
        std::vector<double> &v_t,
        bool is_aux_lidar )
    {
        reset();
        if (is_aux_lidar)
        {
            AuxAviaHandler(msg);
        }
        else
        {
            AviaHandler(msg);
        }
        pcl_out = std::move(v_cloud);
        v_t = std::move(v_timestamp);
    }

    void CloudConvert2::Process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                std::vector<std::vector<point3D>> &pcl_out,
                                std::vector<double> &v_t)
    {
        reset();

        GazeboHandler(msg);
        pcl_out = std::move(v_cloud);
        v_t = std::move(v_timestamp);
    }

    void CloudConvert2::AviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
    {
        const int plsize = std::min<int>(msg->point_num, msg->points.size());
        if (plsize <= 0 || sweep_cut_num <= 0)
        {
            return;
        }

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        timespan_ = msg->points[plsize - 1].offset_time / tm_scale;
        if (timespan_ <= 0.0)
        {
            return;
        }

        delta_time = timespan_ / sweep_cut_num;
        // 每一个小片段结束时的相对时间
        v_timestamp.resize(sweep_cut_num);
        for (int i = 0; i < sweep_cut_num; i++)
            v_timestamp[i] = /*msg->header.stamp.toSec() +*/ (i + 1) * delta_time; //  delta time

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;

            if (msg->points[i].offset_time / tm_scale > timespan_)
            {
                std::cout << "------" << __FUNCTION__ << ", " << __LINE__ << ", error timespan:" << timespan_ << " < " << msg->points[i].offset_time << std::endl;
                continue;
            }
            if (i % point_filter_num_ != 0)
                continue;

            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                           msg->points[i].z * msg->points[i].z;
            if (range > 250 * 250 || range < blind * blind)
                continue;

            if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                point3D point_temp;
                point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                point_temp.point = point_temp.raw_point;
                point_temp.relative_time = msg->points[i].offset_time / tm_scale; // seconds after conversion by tm_scale
                point_temp.intensity = msg->points[i].reflectivity;
                // 绝对时间戳
                point_temp.timestamp = headertime + point_temp.relative_time;

                int id = static_cast<int>((msg->points[i].offset_time / tm_scale) / delta_time); //  get id
                id = std::clamp(id, 0, sweep_cut_num - 1);
                // 当前点在它所属的那个小时间段内的归一化时间比例
                point_temp.alpha_time = point_temp.relative_time / delta_time - id;
                // 记录每个小时间分段的时间跨度
                point_temp.timespan = delta_time;
                point_temp.ring = msg->points[i].line;

                v_cloud[id].push_back(point_temp);
            }
        }
    }

    void CloudConvert2::AuxAviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
    {
        const int plsize = std::min<int>(msg->point_num, msg->points.size());
        if (plsize <= 0)
        {
            return;
        }

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        timespan_ = msg->points[plsize - 1].offset_time / tm_scale;
        if (timespan_ <= 0.0)
        {
            return;
        }

        v_timestamp.resize(1);
        v_timestamp[0] = timespan_;

        for (int i = 0; i < plsize; i++)
        {
            const auto &pt = msg->points[i];

            if (i % point_filter_num_ != 0)
                continue;
            if (!(std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)))
                continue;

            double range = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (range > 250 * 250 || range < blind * blind)
                continue;

            if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                point3D point_temp;
                point_temp.raw_point = Eigen::Vector3d(pt.x, pt.y, pt.z);
                point_temp.point = point_temp.raw_point;
                point_temp.intensity = pt.reflectivity;
                point_temp.ring = pt.line;

                point_temp.relative_time = pt.offset_time / tm_scale;
                point_temp.timestamp = headertime + point_temp.relative_time;

                point_temp.timespan = timespan_;

                point_temp.alpha_time = point_temp.relative_time / timespan_;

                v_cloud[0].push_back(point_temp);
            }
        }
    }

    void CloudConvert2::GazeboHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        // GPU 雷达整帧同时采样，不沿点序号伪造扫描时间，也不读取转换器的 time/ring。
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        timespan_ = 0.0;
        v_cloud.resize(1);
        v_timestamp.assign(1, 0.0);
        auto &output = v_cloud.front();
        output.reserve(cloud.size());

        for (size_t i = 0; i < cloud.size(); i += static_cast<size_t>(point_filter_num_))
        {
            const auto &sample = cloud.points[i];
            const Eigen::Vector3d xyz(sample.x, sample.y, sample.z);
            if (!xyz.allFinite() || xyz.squaredNorm() < blind * blind ||
                xyz.squaredNorm() > 250.0 * 250.0)
            {
                continue;
            }

            point3D point;
            point.raw_point = xyz;
            point.point = xyz;
            point.timestamp = stamp;
            output.push_back(std::move(point));
        }
    }

    void CloudConvert2::reset()
    {
        if (sweep_cut_num <= 0)
        {
            sweep_cut_num = 1;
        }

        if (v_cloud.size() != static_cast<size_t>(sweep_cut_num))
        {
            v_cloud.clear();
            v_cloud.resize(sweep_cut_num);
        }
        else
        {
            for (auto &cloud : v_cloud)
            {
                cloud.clear(); // keep capacity to avoid per-frame reallocation
            }
        }
        v_timestamp.clear();
    }

    void CloudConvert2::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();
        sweep_cut_num = yaml["preprocess"]["sweep_cut_num"].as<int>();
        blind = yaml["preprocess"]["blind"].as<double>();
        if(yaml["preprocess"]["degenerate_window_num"])
            throw std::runtime_error("Retired preprocess.degenerate_window_num; geometry diagnostic is frame-local");


        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::GAZEBO;
            if (sweep_cut_num != 1)
            {
                throw std::runtime_error("Gazebo snapshot requires sweep_cut_num=1");
            }
            LOG(INFO) << "Using Gazebo instantaneous PointCloud2";
        }
        else
        {
            throw std::runtime_error("Unsupported lidar_type: use 1 for Livox or 2 for Gazebo");
        }
        if (point_filter_num_ < 1 || !std::isfinite(blind) || blind < 0.0)
        {
            throw std::runtime_error("Invalid point_filter_num or blind range");
        }
    }

} // namespace zjloc
