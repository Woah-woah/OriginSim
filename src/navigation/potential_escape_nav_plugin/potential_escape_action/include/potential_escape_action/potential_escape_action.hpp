// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_behavior_tree
{

class RegionTimeoutTracker
{
public:
  RegionTimeoutTracker()
  : timeout_(std::chrono::duration<double>(0.0)), triggered_(false), active_(false)
  {
  }

  void setTimeout(double timeout_sec)
  {
    timeout_ = std::chrono::duration<double>(timeout_sec);
  }

  void reset()
  {
    active_ = false;
    triggered_ = false;
  }

  bool update(
    bool in_target_region,
    const std::chrono::steady_clock::time_point & now)
  {
    if (triggered_) {
      return true;
    }

    if (!in_target_region) {
      active_ = false;
      return false;
    }

    if (!active_) {
      entered_at_ = now;
      active_ = true;
      return timeout_.count() <= 0.0;
    }

    if ((now - entered_at_) >= timeout_) {
      triggered_ = true;
    }

    return triggered_;
  }

private:
  std::chrono::duration<double> timeout_;
  std::chrono::steady_clock::time_point entered_at_;
  bool triggered_;
  bool active_;
};

class RegionTimeoutCondition : public BT::ConditionNode
{
public:
  RegionTimeoutCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initialize();
  void createSubscription();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr region_sub_;
  std::mutex region_mutex_;
  std::string region_topic_;
  int target_region_{1};
  int latest_region_{-1};
  bool has_region_{false};
  double timeout_sec_{3.0};
  bool initialized_{false};
  RegionTimeoutTracker tracker_;
};

/**
   * @brief
   * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
   *       It will re-initialize when halted.
   */
class EscapeAction : public BtActionNode<nav2_msgs::action::BackUp>
{

public:
  /**
   * @brief A constructor for nav2_behavior_tree::EscapeAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  EscapeAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  ~EscapeAction();

  void on_tick() override;
  // void on_wait_for_result() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
        BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for reversing"),
        BT::InputPort<double>("looking_radius", 0.5, "Planning radius in meters"),
        BT::InputPort<std::string>("robot_frame", "    ", "your robot frame"),
      });
  }

private:
  std::optional<double> calculate_escape_angle(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double radius,
    const std::string & robot_frame);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_on;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
  std::mutex costmap_mutex_;
  double backup_dist_;
  double speed;
  double radius;
  double time_allowance;
  std::string robot_frame_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_ACTION_HPP_
