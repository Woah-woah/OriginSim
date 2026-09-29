#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FakeBaselink : public rclcpp::Node
{
public:
    FakeBaselink(const rclcpp::NodeOptions & options)
        : Node("fake_baselink", options)
    {
        this->declare_parameter("odom_frame_id","odom");
        this->declare_parameter("base_link_frame_id","base_link");
        this->declare_parameter("base_link_fake_frame_id","base_link_fake");
        this->declare_parameter("cmd_vel_topic_name","cmd_vel");
        this->declare_parameter("cmd_vel_after_topic_name","cmd_vel_chassis");
        this->declare_parameter("forward_distance",0.2);

        this->get_parameter("odom_frame_id",odom_frame_id);
        this->get_parameter("base_link_frame_id",base_link_frame_id);
        this->get_parameter("base_link_fake_frame_id",base_link_fake_frame_id);
        this->get_parameter("cmd_vel_topic_name",cmd_vel_topic_name);
        this->get_parameter("cmd_vel_after_topic_name",cmd_vel_after_topic_name);
        this->get_parameter("forward_distance",forward_distance);

        tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer);
        tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_name,1,std::bind(&FakeBaselink::cmd_vel_callback,this,std::placeholders::_1)
        );

        cmd_vel_after_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_after_topic_name,10
        );

        pub_fake_timer = this->create_wall_timer(
            std::chrono::milliseconds(50),std::bind(&FakeBaselink::pub_fake_callback,this)
        );

        init_base_link_fake();
    }

    void init_base_link_fake()
    {
        geometry_msgs::msg::TransformStamped odom_to_base_link,fake_trans;
        while (1)
        {
            try
            {
                odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
                break;
            }
            catch(...){
            }
        }
        fake_q = odom_to_base_link.transform.rotation;
        base_link_translation = odom_to_base_link.transform.translation;

        fake_trans.header.stamp = this->get_clock()->now();
        fake_trans.header.frame_id = odom_frame_id;
        fake_trans.child_frame_id = base_link_fake_frame_id;

        fake_trans.transform.translation = base_link_translation;
        fake_trans.transform.rotation = fake_q;

        tf2_broadcaster->sendTransform(fake_trans);
        ready = 1;
    }


    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {

        geometry_msgs::msg::TransformStamped odom_to_base_link;
        try
        {
            odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
        }
        catch(...){}

        geometry_msgs::msg::Twist cmd_vel_after;

        double base_link_angle = tf2::getYaw(odom_to_base_link.transform.rotation);
        double angle_diff = base_link_angle - tf2::getYaw(fake_q);

        cmd_vel_after.angular.z = 0.0;
        cmd_vel_after.linear.x = cmd_vel->linear.x * cos(angle_diff) + cmd_vel->linear.y * sin(angle_diff);
        cmd_vel_after.linear.y = -cmd_vel->linear.x * sin(angle_diff) + cmd_vel->linear.y * cos(angle_diff);

        cmd_vel_after_pub->publish(cmd_vel_after);
        
    }

    void pub_fake_callback()
    {
        if (!ready)
            return;

        geometry_msgs::msg::TransformStamped fake_trans,odom_to_base_link;

        try
        {
            odom_to_base_link = tf2_buffer->lookupTransform(odom_frame_id,base_link_frame_id,rclcpp::Time(0));
            base_link_translation = odom_to_base_link.transform.translation;
        }
        catch(...){}

        fake_trans.header.stamp = this->get_clock()->now();
        fake_trans.header.frame_id = odom_frame_id;
        fake_trans.child_frame_id = base_link_fake_frame_id;

        fake_trans.transform.translation = base_link_translation;
        fake_trans.transform.rotation = fake_q;

        tf2_broadcaster->sendTransform(fake_trans);

    }

private:
    std::string odom_frame_id;
    std::string base_link_frame_id;
    std::string base_link_fake_frame_id;
    std::string cmd_vel_topic_name;
    std::string cmd_vel_after_topic_name;
    
    std::int8_t ready;

    float forward_distance;

    geometry_msgs::msg::Quaternion fake_q;
    geometry_msgs::msg::Vector3 base_link_translation;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_after_pub;

    rclcpp::TimerBase::SharedPtr pub_fake_timer;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FakeBaselink)