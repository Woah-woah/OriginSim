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
#ifndef GZ_GUI_RefereeUI_HH_
#define GZ_GUI_RefereeUI_HH_

#include <string>
#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <QSharedMemory>
#include <QTimer>

namespace gz
{
  namespace gui
  {
    class RefereeUI : public Plugin
    {
      Q_OBJECT
      Q_PROPERTY(QString label READ Label NOTIFY LabelChanged);
      Q_PROPERTY(int maxValue READ MaxValue NOTIFY MaxValueChanged);
      Q_PROPERTY(int defaultValue READ DefaultValue NOTIFY defaultValueChanged);

    public:
      RefereeUI();
      virtual ~RefereeUI();
      virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem);
      Q_INVOKABLE QString Label() const { return QString::fromStdString(labelScreen); }
      Q_INVOKABLE int MaxValue() const { return maxValue; }
      Q_INVOKABLE int DefaultValue() const { return defaultValue; }

    signals:
      void LabelChanged();
      void MaxValueChanged();
      void defaultValueChanged();
      void MsgChanged();

    protected slots:
      void OnSlider(int _value);

    private:
      std::string label{"Default Name"};
      std::string labelScreen{"Default Name"};
      int maxValue = 100;
      int defaultValue = 100;
      QTimer *watchTimer = nullptr;
      QSharedMemory shared_memory;
      int lastValue = -1;
      void checkSharedMemory();
    };
  }
}

#endif
