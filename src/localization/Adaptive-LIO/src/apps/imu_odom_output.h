#pragma once
#include "algo/imu_odom_predictor.h"
#include "algo/odom_projection.h"
#include <condition_variable>
#include <atomic>
#include <thread>
#include <mutex>
#include <glog/logging.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>

// Defined in main_ada.cpp; shared by cloud and odometry publishers.
inline rclcpp::Time get_ros_time(double timestamp);

// ROS adapter. Configuration is immutable after Start(); one worker owns the
// predictor and is the ONLY /odom, path and dynamic TF publisher.
class ImuOdomOutput {
public:
    ImuOdomOutput(const nav_msgs::msg::Path& path,
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_pub,
        const rclcpp::Clock::SharedPtr& clock, tf2_ros::TransformBroadcaster* tf,
        const std::string& world, const std::string& child)
        : odom_pub_(odom),path_pub_(path_pub),clock_(clock),tf_(tf),
          world_(world),child_(child),path_(path) {}
    ~ImuOdomOutput(){Stop();}
    void SetAftToBaseTransform(const Vec3d& t,const Vec3d& rpy) {
        t_=t; q_=Eigen::AngleAxisd(rpy.z(),Vec3d::UnitZ())*
                 Eigen::AngleAxisd(rpy.y(),Vec3d::UnitY())*Eigen::AngleAxisd(rpy.x(),Vec3d::UnitX());
    }
    void SetBaseLinkFrame(const std::string& frame){base_=frame;}
    void SetMaxPathPoses(size_t count){max_path_=count;}
    void Start(){worker_=std::thread([this]{
        try {Run();}
        catch(const std::exception& e){Fail();LOG(ERROR)<<"IMU output worker failed: "<<e.what();}
    });}
    void Stop(){
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
        cv_.notify_all();if(worker_.joinable())worker_.join();
    }
    void SetFaultCallback(std::function<void()> callback){fault_callback_=std::move(callback);}
    void Fail(){if(!faulted_.exchange(true) && fault_callback_)fault_callback_();cv_.notify_one();}
    void OnImu(const IMUPtr& imu){
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || faulted_)return;
        if(imu_queue_.size()>=20000){Fail();return;}
        imu_queue_.push_back(*imu);cv_.notify_one();
    }
    void OnState(const zjloc::LioStateUpdate& state){
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || faulted_)return;
        if(!pending_ || state.sequence>pending_->sequence)pending_=state;
        cv_.notify_one();
    }
private:
    void Run(){
        zjloc::ImuOdomPredictor predictor;
        double last_published=-std::numeric_limits<double>::infinity(),last_path=last_published;
        auto last_log=std::chrono::steady_clock::now();
        auto last_input=last_log;
        while(true){
            std::deque<zjloc::IMU> batch;std::optional<zjloc::LioStateUpdate> update;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock,std::chrono::seconds(1),[&]{return stopping_||faulted_||!imu_queue_.empty()||pending_.has_value();});
                if(stopping_)break;
                if(faulted_){LOG(ERROR)<<"IMU output invalid: input reset/gap/queue overflow; restart whole adaptive_lio node";break;}
                batch.swap(imu_queue_);update=std::move(pending_);pending_.reset();
            }
            if(!batch.empty())last_input=std::chrono::steady_clock::now();
            if(std::chrono::steady_clock::now()-last_input>std::chrono::seconds(1))
                LOG(WARNING)<<"IMU input interrupted; no synthetic timestamps or repeated TF published";
            for(const auto& imu:batch)predictor.Append(imu);
            if(update)predictor.Submit(*update);
            predictor.Advance();
            if (predictor.Faulted()) {
                const auto &c = predictor.counters;
                LOG(ERROR) << "IMU replay failed; restart whole adaptive_lio node"
                           << " history_miss=" << c.history_miss << " gap=" << c.gap
                           << " invalid_imu=" << c.invalid_imu << " invalid_state=" << c.invalid_state
                           << " history=" << predictor.HistorySize();
                Fail();
                continue;
            }
            const auto now=std::chrono::steady_clock::now();
            const bool publishable=predictor.Publishable();
            if(now-last_log>std::chrono::seconds(5)){
                const auto& c=predictor.counters;
                LOG(INFO)<<"imu_output valid="<<publishable<<" history="<<predictor.HistorySize()
                         <<" applied="<<c.applied<<" stale_state="<<c.stale_state
                         <<" history_miss="<<c.history_miss<<" timeout="<<c.timeout
                         <<" degraded="<<predictor.Degraded()<<" degraded_total_s="<<predictor.DegradedSeconds()
                         <<" batch="<<batch.size();last_log=now;
            }
            if(!publishable)continue;
            const auto state=predictor.Snapshot();
            const double stamp=state.nav.timestamp_;
            if(stamp<=last_published || faulted_)continue;
            const auto projected=zjloc::ProjectOdometry(state,predictor.AngularVelocity(),predictor.GyroVariance());
            nav_msgs::msg::Odometry odom;
            odom.header.stamp=get_ros_time(stamp);odom.header.frame_id=world_;odom.child_frame_id=child_;
            const auto& pos=projected.pose.translation();const auto q=projected.pose.unit_quaternion();
            odom.pose.pose.position.x=pos.x();odom.pose.pose.position.y=pos.y();odom.pose.pose.position.z=pos.z();
            odom.pose.pose.orientation.x=q.x();odom.pose.pose.orientation.y=q.y();
            odom.pose.pose.orientation.z=q.z();odom.pose.pose.orientation.w=q.w();
            odom.twist.twist.linear.x=projected.linear.x();odom.twist.twist.linear.y=projected.linear.y();odom.twist.twist.linear.z=projected.linear.z();
            odom.twist.twist.angular.x=projected.angular.x();odom.twist.twist.angular.y=projected.angular.y();odom.twist.twist.angular.z=projected.angular.z();
            for(int r=0;r<6;++r)for(int c=0;c<6;++c){
                odom.pose.covariance[r*6+c]=projected.pose_cov(r,c);
                odom.twist.covariance[r*6+c]=projected.twist_cov(r,c);
            }
            odom_pub_->publish(odom);

            geometry_msgs::msg::TransformStamped tf;
            tf.header=odom.header;tf.child_frame_id=child_;tf.transform.translation.x=pos.x();
            tf.transform.translation.y=pos.y();tf.transform.translation.z=pos.z();tf.transform.rotation=odom.pose.pose.orientation;
            
            // 发布aft_mapped 2 baselink
            geometry_msgs::msg::TransformStamped fixed;
            fixed.header=odom.header;fixed.header.frame_id=child_;fixed.child_frame_id=base_;
            fixed.transform.translation.x=t_.x();fixed.transform.translation.y=t_.y();fixed.transform.translation.z=t_.z();
            fixed.transform.rotation.x=q_.x();fixed.transform.rotation.y=q_.y();fixed.transform.rotation.z=q_.z();fixed.transform.rotation.w=q_.w();
            // if(tf_)tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{tf,fixed});
            if(tf_)tf_->sendTransform(tf);

            last_published=stamp;
            if(max_path_ && path_pub_->get_subscription_count()>0 && stamp-last_path>=.1){
                geometry_msgs::msg::PoseStamped ps;ps.header=odom.header;ps.pose=odom.pose.pose;
                path_.header=odom.header;path_.poses.push_back(ps);
                if(path_.poses.size()>max_path_)path_.poses.erase(path_.poses.begin(),path_.poses.end()-max_path_);
                path_pub_->publish(path_);last_path=stamp;
            }
            if(update && now-last_state_log_>std::chrono::seconds(5)) {
                last_state_log_=now; LOG(INFO)<<"posterior_replay_age_s="<<stamp-update->state.nav.timestamp_
                              <<" correction_age_s="<<stamp-predictor.LastAccepted()
                              <<" posterior_stamp="<<update->state.nav.timestamp_;
            }
        }
    }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Clock::SharedPtr clock_;
    tf2_ros::TransformBroadcaster* tf_;
    std::string world_,child_,base_="base_link";
    Vec3d t_=Vec3d::Zero();Eigen::Quaterniond q_=Eigen::Quaterniond::Identity();
    nav_msgs::msg::Path path_;size_t max_path_=300;
    std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
    std::deque<zjloc::IMU> imu_queue_;std::optional<zjloc::LioStateUpdate> pending_;
    std::function<void()> fault_callback_;
    std::chrono::steady_clock::time_point last_state_log_{};
    bool stopping_=false;std::atomic<bool> faulted_{false};
};
