#include <iostream>
#include <gz/plugin/Register.hh>
#include <string>
#include "RefereeUI.hh"
#include <cstring>
#include <QSharedMemory>
#include <QTimer>

using namespace gz;
using namespace gui;

RefereeUI::RefereeUI() : Plugin()
{
    this->watchTimer = new QTimer(this);
    connect(this->watchTimer, &QTimer::timeout, this, &RefereeUI::checkSharedMemory);
    this->watchTimer->start(100);  // 100ms 轮询
}

RefereeUI::~RefereeUI()
{
    if (shared_memory.isAttached())
    {
        shared_memory.detach();
    }
}

void RefereeUI::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
    if (!_pluginElem)
        return;

    auto labelElem = _pluginElem->FirstChildElement("label");
    auto msgElem = _pluginElem->FirstChildElement("msg");
    auto maxValue = _pluginElem->FirstChildElement("maxValue");
    auto defaultValue = _pluginElem->FirstChildElement("default");
    this->label = labelElem->GetText();

    if (labelElem && labelElem->GetText())
    {
        if (maxValue && maxValue->GetText())
        {
            this->maxValue = std::stod(maxValue->GetText());
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  " + std::string(maxValue->GetText())).c_str();
        }
        else
        {
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  100").c_str();
        }
        emit LabelChanged();
        emit MaxValueChanged();
    }

    if (defaultValue && defaultValue->GetText())
    {
        this->defaultValue = std::stod(defaultValue->GetText());
    }
    else
    {
        this->defaultValue = this->maxValue;
    }
    emit defaultValueChanged();

    // 创建共享内存
    if (shared_memory.isAttached())
    {
        shared_memory.detach();
    }

    if (msgElem && msgElem->GetText())
    {
        shared_memory.setKey(QString::fromStdString(std::string(msgElem->GetText())));
    }
    else
    {
        shared_memory.setKey(QString::fromStdString(std::string(labelElem->GetText())));
    }

    if (!shared_memory.create(sizeof(int)))
    {
        std::cerr << "Failed to create shared memory: " << shared_memory.errorString().toStdString() << std::endl;
    }

    std::cout << std::string(labelElem->GetText()) << "\tMax Value:\t" << this->maxValue << "\tDefault Value:\t" << this->defaultValue << std::endl;

    // 设置默认值
    shared_memory.lock();
    memcpy(shared_memory.data(), &(this->defaultValue), sizeof(int));
    shared_memory.unlock();

    this->lastValue = this->defaultValue;  // 初始化 lastValue
}

void RefereeUI::OnSlider(int _value)
{
    if (shared_memory.isAttached())
    {
        shared_memory.lock();
        memcpy(shared_memory.data(), &_value, sizeof(int));
        shared_memory.unlock();
    }
}


void RefereeUI::checkSharedMemory()
{
    if (!shared_memory.isAttached())
        return;

    shared_memory.lock();
    int currentValue = 0;
    memcpy(&currentValue, shared_memory.constData(), sizeof(int));
    shared_memory.unlock();

    if (currentValue != this->lastValue)
    {
        this->lastValue = currentValue;
        this->defaultValue = currentValue;
        emit defaultValueChanged();
    }
}

// Register plugin
GZ_ADD_PLUGIN(RefereeUI, gz::gui::Plugin)
