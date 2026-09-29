/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_GUI_RaceControl_HH_
#define GZ_GUI_RaceControl_HH_

#include <string>
#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <QSharedMemory>
#include <QTimer>
#include <cstring>
#include <QSharedMemory>
#include <tf2/exceptions.h>
#include <iostream>
#include <string>


// ros/RFID
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>


struct RegionEffect {
    std::string targetKey;
    std::string operation;
    int value;
};

struct Region {
    std::string name;
    bool active;
    std::vector<geometry_msgs::msg::Point> points; 
    RegionEffect effect;
};


namespace gz
{
  namespace gui
  {
    class RaceControl : public Plugin
    {
      Q_OBJECT
      Q_PROPERTY(QString label READ Label NOTIFY LabelChanged);
      Q_PROPERTY(int maxValue READ MaxValue NOTIFY MaxValueChanged);
      Q_PROPERTY(int currentTime READ CurrentTime NOTIFY CurrentTimeChanged);
      Q_PROPERTY(int bulletablesend READ BulletAbleSend NOTIFY BulletAbleSendChanged);
      Q_PROPERTY(QString time READ Time NOTIFY TimeChanged);

    public:
      RaceControl();
      virtual ~RaceControl();
      virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem);
      void loadRegions(const std::string& filePath);
      Q_INVOKABLE QString Label() const { return QString::fromStdString(labelScreen); }
      Q_INVOKABLE QString Time() const { return QString::fromStdString(simTimeStr); }
      Q_INVOKABLE int MaxValue() const { return maxValue; }
      Q_INVOKABLE int CurrentTime() const { return currentTime; }

      Q_INVOKABLE int BulletAbleSend() const { return bulletablesend; }
      Q_INVOKABLE void resetSlider();
      Q_INVOKABLE void sendBullet();
      Q_INVOKABLE void startRace();
      Q_INVOKABLE void stopRace();

    signals:
      void LabelChanged();
      void MaxValueChanged();
      void CurrentTimeChanged();
      void MsgChanged();
      void TimeChanged();
      void BulletAbleSendChanged();

    protected slots:
      void OnSlider(int _value);

    private:
      void onTimerTimeout();
      void updateBullet(int bulletosend, int flag);
      bool is_in_region(const geometry_msgs::msg::Point &pt, const Region &region);
      void applyMemoryEffect(const RegionEffect &effect);

      // RFID:ROS 2 节点与 TF
      std::vector<Region> regions;
      rclcpp::Node::SharedPtr ros_node_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      geometry_msgs::msg::Point curr_pos_;

      // RACECONTROL
      std::string label{"Default Name"};
      std::string labelScreen{"Default Name"};
      std::string simTimeStr{"07:00"};
      int maxValue = 100;
      int defaultValue = 100;
      int currentTime = 100;
      int bulletablesend = 0;
      int bulletsended = 0;
      QSharedMemory timeMemory;
      QSharedMemory bulletMemory;
      std::string timeKey;
      QTimer *timer;
    };
  }
}

#endif
