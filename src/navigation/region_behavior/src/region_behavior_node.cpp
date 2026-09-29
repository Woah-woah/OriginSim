#include "region_behavior/region_behavior_node.hpp"

namespace region_behavior
{

RegionBehaviorNode::RegionBehaviorNode() : Node("region_behavior_node")
{
    this->declare_parameter("map_frame_id", "map");
    this->get_parameter("map_frame_id", map_frame_id_);
    this->declare_parameter("base_link_frame_id", "base_link");
    this->get_parameter("base_link_frame_id", base_link_frame_id_);

    std::string pkg_dir = ament_index_cpp::get_package_share_directory("region_behavior");
    std::string default_path = pkg_dir + "/config/0409reg.yaml";
    this->declare_parameter("yaml_path", default_path);
    this->get_parameter("yaml_path", yaml_path_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (!load_regions(yaml_path_))
    {
        RCLCPP_ERROR(this->get_logger(), "区域配置加载失败，节点退出。");
        rclcpp::shutdown();
        return;
    }

    region_int_pub_ = this->create_publisher<std_msgs::msg::Int32>("/region_int", 10);
    gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalRegionCmd>("/serial/gimbal_region_cmd", 10);

    region_area_pub_ = this->create_publisher<rm_interfaces::msg::RegionArea>(
        "/region_area", 1);
    uphill_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/serial/uphill", 10);
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(
        "/speed_limit", 10);
    arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "/direction_arrow", 1);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RegionBehaviorNode::timer_callback, this));

    get_region_service_ = this->create_service<region_behavior::srv::GetRegion>(
        "/region_behavior/get_region",
        std::bind(&RegionBehaviorNode::handle_get_region, this, std::placeholders::_1, std::placeholders::_2));
    area_pub_msg_.area_type = rm_interfaces::msg::RegionArea::OUR;
    RCLCPP_INFO(this->get_logger(), "节点启动完成。");
}
bool RegionBehaviorNode::load_regions(const std::string &yaml_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (!config["regions"])
        {
            RCLCPP_ERROR(this->get_logger(), "YAML文件中未找到 'regions' 节点");
            return false;
        }

        for (const auto &region_node : config["regions"])
        {
            Region region;
            region.id = region_node["id"].as<std::string>();
            region.type = region_node["type"].as<int>();

            for (const auto &p_node : region_node["points"])
            {
                geometry_msgs::msg::Point pt;
                pt.x = p_node["x"].as<double>();
                pt.y = p_node["y"].as<double>();
                pt.z = p_node["z"].as<double>();
                region.points.push_back(pt);
            }

            for (auto it = region_node.begin(); it != region_node.end(); ++it)
            {
                std::string key = it->first.as<std::string>();

                if (key == "id" || key == "type" || key == "points")
                {
                    continue;
                }

                try
                {
                    region.custom_keys[key] = it->second.as<std::string>();
                }
                catch (const YAML::Exception &e)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "区域 %s 的自定义 key %s 无法转成 string，已跳过: %s",
                                region.id.c_str(),
                                key.c_str(),
                                e.what());
                }
            }

            region.long_edge = compute_long_edge(region.points);
            regions_.push_back(region);
            RCLCPP_INFO(this->get_logger(), "加载区域成功: %s (顶点数: %zu)",
                        region.id.c_str(), region.points.size());
        }
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "加载YAML文件出错: %s", e.what());
        return false;
    }

    return !regions_.empty();
}

bool RegionBehaviorNode::is_in_region(const geometry_msgs::msg::Point &pt, const Region &region)
{
    bool inside = false;
    size_t n = region.points.size();

    for (size_t i = 0; i < n; ++i)
    {
        const auto &a = region.points[i];
        const auto &b = region.points[(i + 1) % n];

        // 判断点是否在边界上
        if ((pt.x == a.x && pt.y == a.y) ||
            (pt.x == b.x && pt.y == b.y))
            return true;

        if (((a.y > pt.y) != (b.y > pt.y)) &&
            (pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x))
        {
            inside = !inside;
        }
    }
    return inside;
}

int RegionBehaviorNode::get_region_int(const geometry_msgs::msg::Point &pt, Region &region)
{
    for (const auto &region_ : regions_)
    {
        if (is_in_region(pt, region_))
        {
            region = region_;
            return region.type;
        }
    }
    return -1;
}


void RegionBehaviorNode::start_bumpy_session(
    const Region &region,
    const Eigen::Vector2d &curr_pos_vect,
    const geometry_msgs::msg::TransformStamped &tf)
{
    Eigen::Vector2d A = region.long_edge.start;
    Eigen::Vector2d B = region.long_edge.end;

    // 当前点更靠近哪一端，就认为从哪一端进入
    if ((curr_pos_vect - A).squaredNorm() < (curr_pos_vect - B).squaredNorm())
    {
        entry_end_ = A;
        exit_end_ = B;
        pass_direction_ = (B - A).normalized();
        RCLCPP_INFO(this->get_logger(), "[颠簸区] 入口侧=A，出口侧=B");
    }
    else
    {
        entry_end_ = B;
        exit_end_ = A;
        pass_direction_ = (A - B).normalized();
        RCLCPP_INFO(this->get_logger(), "[颠簸区] 入口侧=B，出口侧=A");
    }

    total_len_ = (exit_end_ - entry_end_).norm();
    max_progress_in_region_ = 0.0;
    last_progress_ = 0.0;
    last_inside_pos_ = curr_pos_vect;
    bumpy_session_active_ = true;
    current_bumpy_passed_ = false;
    current_bumpy_retreated_ = false;
    auto it = region.custom_keys.find("side");
    if (it != region.custom_keys.end()) {
        current_bumpy_region_area_ = it->second;
    }

    publish_rectangle_debug_marker(region, pass_direction_, curr_pos_);

    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    double roll, pitch, robot_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw);

    double dir_yaw = std::atan2(pass_direction_.y(), pass_direction_.x());
    double angle = dir_yaw - robot_yaw;

    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
    {
        angle += 2.0 * M_PI;
    }
    angle -= M_PI;
    in_region_angle = angle * 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(),
                "[颠簸区] 开始一次通过判定: total_len=%.3f m, in_region_angle=%.2f deg",
                total_len_, in_region_angle);
}

void RegionBehaviorNode::update_bumpy_progress(const Eigen::Vector2d &curr_pos_vect)
{
    if (!bumpy_session_active_ || total_len_ < 1e-6)
    {
        return;
    }

    double progress = (curr_pos_vect - entry_end_).dot(pass_direction_);

    // 限制到 [0, total_len_]
    progress = std::clamp(progress, 0.0, total_len_);

    last_progress_ = progress;
    max_progress_in_region_ = std::max(max_progress_in_region_, progress);
    last_inside_pos_ = curr_pos_vect;
}

void RegionBehaviorNode::finish_bumpy_session()
{
    if (!bumpy_session_active_)
    {
        return;
    }

    const double pass_ratio = (total_len_ > 1e-6) ? (max_progress_in_region_ / total_len_) : 0.0;

    double d_entry = (last_inside_pos_ - entry_end_).norm();
    double d_exit = (last_inside_pos_ - exit_end_).norm();
    bool near_exit_side = d_exit < d_entry;

    
    // 1) 最大进度超过总长度的80%
    // 2) 离开前最后一个区域内位置更接近出口侧
    if (pass_ratio > 0.7 && near_exit_side)
    {
        current_bumpy_passed_ = true;
        current_bumpy_retreated_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "[颠簸区] 判定结果：完整通过。pass_ratio=%.3f, d_entry=%.3f, d_exit=%.3f",
                    pass_ratio, d_entry, d_exit);

        /* ========================以下为颠簸路段特化===============================*/
        geometry_msgs::msg::TransformStamped tf;

        tf = tf_buffer_->lookupTransform(map_frame_id_, base_link_frame_id_, tf2::TimePointZero);
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        double roll, pitch, robot_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw);


        if (current_bumpy_region_area_ == "ourside")
        {
            if (abs(robot_yaw)< 90.0)
            { // 向中央高地
                area_pub_msg_.area_type = rm_interfaces::msg::RegionArea::MID;
            }
            else
            { // 从中央高低回来
                std::cout << "ourside" << std::endl;
                area_pub_msg_.area_type = rm_interfaces::msg::RegionArea::OUR;
            }
        }
        else if ( current_bumpy_region_area_ == "theirside")
        {
            if (abs(robot_yaw)< 90.0)
            { // 向敌方基地
                area_pub_msg_.area_type = rm_interfaces::msg::RegionArea::THEIR;
            }
            else
            { // 向中央高低
                area_pub_msg_.area_type = rm_interfaces::msg::RegionArea::MID;
            }

        }
        current_bumpy_region_area_ = "";
        /* ========================以上为颠簸路段特化===============================*/


    }
    else
    {
        current_bumpy_passed_ = false;
        current_bumpy_retreated_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "[颠簸区] 判定结果：中途退出/退回。pass_ratio=%.3f, d_entry=%.3f, d_exit=%.3f",
                    pass_ratio, d_entry, d_exit);
    }

    bumpy_session_active_ = false;
    max_progress_in_region_ = 0.0;
    last_progress_ = 0.0;
    total_len_ = 0.0;
    entry_end_ = Eigen::Vector2d::Zero();
    exit_end_ = Eigen::Vector2d::Zero();
    pass_direction_ = Eigen::Vector2d::Zero();
    last_inside_pos_ = Eigen::Vector2d::Zero();
    
}

void RegionBehaviorNode::timer_callback()
{
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer_->lookupTransform(map_frame_id_, base_link_frame_id_, tf2::TimePointZero);
        curr_pos_.x = tf.transform.translation.x;
        curr_pos_.y = tf.transform.translation.y;
        curr_pos_.z = tf.transform.translation.z;
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", e.what());
        return;
    }

    Region region_;
    Eigen::Vector2d curr_pos_vect(curr_pos_.x, curr_pos_.y);
    int region_int = get_region_int(curr_pos_, region_);

    bool leave_bumpy = (last_region_int_ == 1 && region_int != 1);

    auto msg = std_msgs::msg::Int32();
    msg.data = region_int;
    region_int_pub_->publish(msg);

    region_cmd_.chassis_mode = 2;
    region_cmd_.pass_special_region = 0;
    region_cmd_.pass_region_angle = 0.0;
    uphill_cmd_.data = false;
    nav2_msgs::msg::SpeedLimit speed_msg;

    // 若刚离开颠簸区，先做判定，再清普通状态
    if (leave_bumpy)
    {
        finish_bumpy_session();
        in_region_ = false;
        waiting_times = 0;
        direction = Eigen::Vector2d::Zero();
    }

    switch (region_int)
    {
    case -1: // 没进任何特殊区域
        in_region_ = false;
        waiting_times = 0;
        direction = Eigen::Vector2d::Zero();
        break;
    case 1: // 颠簸路段
    {
        // 如果没有让哨兵停止小陀螺，则停止
        if (waiting_times <= exp_waiting_times)
        {
            region_cmd_.chassis_mode = 1;
            region_cmd_.pass_special_region = 1;
            region_cmd_.pass_region_angle = 500.0; // 状态，表示需要停止小陀螺
            speed_msg.percentage = false;
            speed_msg.speed_limit = 0.5;
            speed_limit_pub_->publish(speed_msg);
            waiting_times++;
            break;
        }
        // 从没进到进
        if (!in_region_)
        {
            RCLCPP_INFO(this->get_logger(), "[颠簸区] 首次进入，建立通过会话");
            start_bumpy_session(region_, curr_pos_vect, tf);
            in_region_ = true;
            direction = pass_direction_;

        }

        update_bumpy_progress(curr_pos_vect);
        

        region_cmd_.chassis_mode = 1;
        region_cmd_.pass_special_region = 1;
        region_cmd_.pass_region_angle = in_region_angle;

        speed_msg.percentage = false;
        speed_msg.speed_limit = 1.5;
        speed_limit_pub_->publish(speed_msg);

        // 可选：输出当前进度，方便调试
        if (bumpy_session_active_ && total_len_ > 1e-6)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[颠簸区] progress=%.3f / %.3f (%.1f%%), max=%.3f",
                         last_progress_, total_len_,
                         100.0 * last_progress_ / total_len_,
                         max_progress_in_region_);
        }

        break;
    }

    case 2: // 上坡发超电
        uphill_cmd_.data = true;
        break;

    default:
        break;
    }
    /* start: 每tick均要发送 */
    uphill_pub_->publish(uphill_cmd_);
    gimbal_cmd_pub_->publish(region_cmd_);
    region_area_pub_->publish(area_pub_msg_);
    /* end:   每tick均要发送 */

    // 记录上一拍区域类型
    last_region_int_ = region_int;
}

void RegionBehaviorNode::publish_rectangle_debug_marker(
    const Region &region,
    const Eigen::Vector2d &direction,
    const geometry_msgs::msg::Point &p)
{
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = map_frame_id_;
    mk.header.stamp = now();
    mk.ns = "rect_debug";
    mk.id = 1;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.05;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    for (auto &pt : region.points)
        mk.points.push_back(pt);
    mk.points.push_back(region.points[0]);
    arrow_pub->publish(mk);

    visualization_msgs::msg::Marker arrow;
    arrow.header = mk.header;
    arrow.ns = "rect_arrow";
    arrow.id = 2;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.1;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start = p;
    geometry_msgs::msg::Point end;
    end.x = start.x + direction.x();
    end.y = start.y + direction.y();
    end.z = start.z;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow_pub->publish(arrow);

    visualization_msgs::msg::Marker sphere;
    sphere.header = mk.header;
    sphere.id = 3;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
    sphere.color.r = 0;
    sphere.color.g = 0;
    sphere.color.b = 1;
    sphere.color.a = 1;
    sphere.pose.position = p;

    arrow_pub->publish(sphere);
}

LongEdge RegionBehaviorNode::compute_long_edge(const std::vector<geometry_msgs::msg::Point> &pts)
{
    if (pts.size() != 4)
    {
        return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }

    Eigen::Vector2d A(pts[0].x, pts[0].y);
    Eigen::Vector2d B(pts[1].x, pts[1].y);
    Eigen::Vector2d C(pts[2].x, pts[2].y);
    Eigen::Vector2d D(pts[3].x, pts[3].y);

    std::array<LongEdge, 4> edges = {
        LongEdge{A, B},
        LongEdge{B, C},
        LongEdge{C, D},
        LongEdge{D, A}};

    double max_len = -1.0;
    LongEdge max_edge;

    for (const auto &e : edges)
    {
        double len = (e.end - e.start).norm();
        if (len > max_len)
        {
            max_len = len;
            max_edge = e;
        }
    }

    return max_edge;
}

void RegionBehaviorNode::handle_get_region(
    const std::shared_ptr<region_behavior::srv::GetRegion::Request> request,
    std::shared_ptr<region_behavior::srv::GetRegion::Response> response)
{
    Region temp_region;
    for (const auto &pt : request->locations)
    {
        int id = get_region_int(pt, temp_region);
        RCLCPP_INFO(this->get_logger(), "查询点 (%.2f, %.2f) 所在区域类型: %d", pt.x, pt.y, id);
        response->region_ids.push_back(id);
    }
}

} // namespace region_behavior

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<region_behavior::RegionBehaviorNode>());
    rclcpp::shutdown();
    return 0;
}