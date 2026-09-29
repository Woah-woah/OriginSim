#ifndef REGION_BEHAVIOR_NODE_HPP_
#define REGION_BEHAVIOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rm_interfaces/msg/gimbal_region_cmd.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include "region_behavior/srv/get_region.hpp"
#include "rm_interfaces/msg/region_area.hpp"
#include <unordered_map>
namespace region_behavior
{

struct LongEdge
{
    Eigen::Vector2d start;
    Eigen::Vector2d end;
};

struct Region
{
    std::string id;
    int type;
    std::vector<geometry_msgs::msg::Point> points;
    LongEdge long_edge;
    std::unordered_map<std::string, std::string> custom_keys;
};

class RegionBehaviorNode : public rclcpp::Node
{
public:
    RegionBehaviorNode();
    ~RegionBehaviorNode() = default;

private:
    bool load_regions(const std::string &yaml_path);
    bool is_in_region(const geometry_msgs::msg::Point &pt, const Region &region);
    int get_region_int(const geometry_msgs::msg::Point &pt, Region &region);
    void timer_callback();
    void publish_rectangle_debug_marker(
        const Region &region,
        const Eigen::Vector2d &direction,
        const geometry_msgs::msg::Point &p);
    LongEdge compute_long_edge(const std::vector<geometry_msgs::msg::Point> &pts);
    void handle_get_region(
        const std::shared_ptr<region_behavior::srv::GetRegion::Request> request,
        std::shared_ptr<region_behavior::srv::GetRegion::Response> response);

    std::vector<Region> regions_;
    std::string map_frame_id_;
    std::string base_link_frame_id_;
    std::string yaml_path_;
    Eigen::Vector2d direction;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub;
    rclcpp::Publisher<rm_interfaces::msg::GimbalRegionCmd>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Publisher<rm_interfaces::msg::RegionArea>::SharedPtr region_area_pub_;

    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr uphill_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr region_int_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<region_behavior::srv::GetRegion>::SharedPtr get_region_service_;
    
    rm_interfaces::msg::GimbalRegionCmd region_cmd_;
    std_msgs::msg::Bool uphill_cmd_;
    geometry_msgs::msg::Point curr_pos_;
    
    bool in_region_ = false;
    int waiting_times = 0;
    int exp_waiting_times = 50;
    float in_region_angle = 0.0;



    // ========= 穿越判定相关状态初始化 =========
    void start_bumpy_session(
        const Region &region,
        const Eigen::Vector2d &curr_pos_vect,
        const geometry_msgs::msg::TransformStamped &tf);

    void update_bumpy_progress(const Eigen::Vector2d &curr_pos_vect);

    void finish_bumpy_session();

    int last_region_int_{-1};

    bool bumpy_session_active_{false};
    bool current_bumpy_passed_{false};
    bool current_bumpy_retreated_{false};
    std::string current_bumpy_region_area_{""};

    rm_interfaces::msg::RegionArea area_pub_msg_;

    Eigen::Vector2d entry_end_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d exit_end_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d pass_direction_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d last_inside_pos_{Eigen::Vector2d::Zero()};

    double total_len_{0.0};
    double max_progress_in_region_{0.0};
    double last_progress_{0.0};
    // ===========================================

};

} // namespace region_behavior

#endif // REGION_BEHAVIOR_NODE_HPP_
