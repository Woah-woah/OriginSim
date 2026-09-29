#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "referee_msg/msg/referee.hpp"
#include <QSharedMemory>
// extern std::map<std::string, int> referee_dict

class referee_publisher : public rclcpp::Node
{
public:
  referee_publisher();
  ~referee_publisher();

private:
  void memory(std::string msg_name);
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<referee_msg::msg::Referee>::SharedPtr publisher_;
  
  QSharedMemory shared_memory;
  size_t count_;
};
