/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#pragma once

// ROS2 message headers
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tools/point_types.h"
#include "common/cloudMap.hpp"

#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace zjloc
{

    /**
     * 预处理雷达点云
     *
     * 将 Livox 原始消息或 Gazebo 快照转为带采样时间的点云
     * 该类由MessageSync类持有，负责将收到的雷达消息与IMU同步并预处理后，再交给LO/LIO算法
     */
    class CloudConvert2
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum class LidarType
        {
            AVIA = 1,    // Livox 实车原始消息
            GAZEBO = 2,  // Gazebo GPU 雷达瞬时点云

        };

        CloudConvert2() = default;
        ~CloudConvert2() = default;

        CloudConvert2(const CloudConvert2 &) = delete;
        CloudConvert2 &operator=(const CloudConvert2 &) = delete;


        /**
         * 处理livox avia 点云
         * @param msg
         * @param pcl_out
         */
    void Process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg,
        std::vector<std::vector<point3D>> &pcl_out,
        std::vector<double> &v_t,
        bool is_aux_lidar
        );

        /**
         * 处理sensor_msgs::PointCloud2点云
         * @param msg
         * @param pcl_out
         */
    void Process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
             std::vector<std::vector<point3D>> &pcl_out,
             std::vector<double> &v_t);

        /// 从YAML中读取参数
        void LoadFromYAML(const std::string &yaml);

        //  返回激光的时间
        double getTimeSpan() { return timespan_; }

        LidarType lidar_type_ = LidarType::AVIA; // 雷达类型

    private:
        void reset();
    void AviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
    void AuxAviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
    void GazeboHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        std::vector<double> v_timestamp;
        std::vector<point3D> cloud_out_;           //  输出点云
        std::vector<std::vector<point3D>> v_cloud; //  out cloud


        int point_filter_num_ = 1; // 跳点
        double blind = 0.1;
        int sweep_cut_num = 1.0;
        double delta_time = 0.1;

        double timespan_;
    };
} // namespace zjloc
