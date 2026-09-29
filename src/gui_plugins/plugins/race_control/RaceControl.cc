#include "RaceControl.hh"
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace gui;

RaceControl::RaceControl() : Plugin(), timeMemory(), bulletMemory()
{
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &RaceControl::onTimerTimeout);
}
RaceControl::~RaceControl()
{
    if (timeMemory.isAttached())
        timeMemory.detach();
    if (bulletMemory.isAttached())
        bulletMemory.detach();
}

void RaceControl::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    if (!ros_node_)
    {
        try
        {
            // 使用标准的 make_shared
            ros_node_ = std::make_shared<rclcpp::Node>("race_control_gui_node");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(ros_node_->get_logger(), "RaceControl ROS Node initialized successfully.");
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to create ROS 2 node: " << e.what() << std::endl;
        }
    }
    if (!_pluginElem)
        return;

    auto labelElem = _pluginElem->FirstChildElement("label");
    auto msgElem = _pluginElem->FirstChildElement("msg");
    auto maxValue = _pluginElem->FirstChildElement("maxValue");
    auto defaultValue = _pluginElem->FirstChildElement("default");
    auto filePath = _pluginElem->FirstChildElement("RFIDFilePath");
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
        this->defaultValue = std::stod(defaultValue->GetText());
    else
        this->defaultValue = this->maxValue;

    currentTime = this->defaultValue;

    emit CurrentTimeChanged();
    emit TimeChanged();
    emit BulletAbleSendChanged();

    if (filePath && filePath->GetText())
    {
        this->loadRegions(filePath->GetText());
    }

    // Setup time memory
    if (timeMemory.isAttached())
        timeMemory.detach();

    timeKey = msgElem && msgElem->GetText() ? msgElem->GetText() : labelElem->GetText();
    timeMemory.setKey(QString::fromStdString(timeKey));

    if (!timeMemory.create(sizeof(int)))
    {
        std::cerr << "Failed to create time shared memory: " << timeMemory.errorString().toStdString() << std::endl;
    }
    else
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &(this->defaultValue), sizeof(int));
        timeMemory.unlock();
    }

    std::cout << std::string(labelElem->GetText()) << "\tMax Value:\t" << this->maxValue << "\tDefault Value:\t" << this->defaultValue << std::endl;
}

void RaceControl::OnSlider(int _value)
{
    int minutes = _value / 60;
    int seconds = _value % 60;
    this->currentTime = _value;
    this->bulletablesend = ((6 - minutes) > 0 ? (6 - minutes) : 0) * 100 - this->bulletsended;

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02d:%02d", minutes, seconds);
    this->simTimeStr = buffer;

    if (timeMemory.isAttached())
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &_value, sizeof(int));
        timeMemory.unlock();
    }

    emit TimeChanged();
    emit CurrentTimeChanged();
    emit BulletAbleSendChanged();
}

void RaceControl::resetSlider()
{

    this->simTimeStr = {"07:00"};
    this->bulletablesend = 0;
    this->bulletsended = 0;
    this->currentTime = this->defaultValue;
    if (!timeMemory.isAttached())
    {
        timeMemory.setKey(QString::fromStdString(this->timeKey));
        if (!timeMemory.attach())
        {
            std::cerr << "[RaceControl] Failed to attach to time memory: "
                      << timeMemory.errorString().toStdString() << std::endl;
        }
    }
    if (timeMemory.isAttached())
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &(this->defaultValue), sizeof(int));
        timeMemory.unlock();
        std::cout << "[RaceControl] Reset shared memory to default value: "
                  << this->defaultValue << std::endl;
    }

    updateBullet(300, 0);

    emit BulletAbleSendChanged();
    emit TimeChanged();
    emit CurrentTimeChanged();
}

void RaceControl::sendBullet()
{
    updateBullet(this->bulletablesend, 1);
    emit BulletAbleSendChanged();
}

void RaceControl::updateBullet(int bulletosend, int flag)
{
    bulletMemory.setKey("bullet_remaining_num_17mm");
    if (!bulletMemory.isAttached())
    {
        if (!bulletMemory.attach())
        {
            std::cerr << "[RaceControl] Failed to attach to bullet memory: "
                      << bulletMemory.errorString().toStdString() << std::endl;
            return;
        }
    }

    if (!bulletMemory.constData())
    {
        std::cerr << "[RaceControl] Bullet memory constData is null!" << std::endl;
        return;
    }

    bulletMemory.lock();
    int sendingbullet = bulletosend + (flag ? *reinterpret_cast<const int *>(bulletMemory.constData()) : 0);
    memcpy(bulletMemory.data(), &(sendingbullet), sizeof(int));
    bulletMemory.unlock();

    this->bulletsended += this->bulletablesend;
    this->bulletablesend = 0;
}

void RaceControl::startRace()
{
    if (!timer->isActive())
    {
        timer->start(1000);
    }
}

void RaceControl::stopRace()
{
    timer->stop();
}

// RFID
bool RaceControl::is_in_region(const geometry_msgs::msg::Point &pt, const Region &region)
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

void RaceControl::loadRegions(const std::string &filePath)
{
    std::cout << "[RaceControl] Loading YAML config from: " << filePath << std::endl;

    try
    {
        // 1. 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(filePath);

        if (!config["regions"])
        {
            std::cerr << "[RaceControl] YAML file missing 'regions' node!" << std::endl;
            return;
        }

        this->regions.clear();

        // 2. 遍历 regions 列表
        for (const auto &node : config["regions"])
        {
            Region region;

            // 读取基本属性 (使用 .as<类型>() 进行转换)
            region.name = node["name"].as<std::string>();
            region.active = node["active"].as<bool>();

            // 3. 读取多边形顶点 points
            if (node["points"] && node["points"].IsSequence())
            {
                for (const auto &pNode : node["points"])
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = pNode["x"].as<double>();
                    pt.y = pNode["y"].as<double>();
                    pt.z = 0.0;
                    region.points.push_back(pt);
                }
            }

            // 4. 读取 effect
            if (node["effect"])
            {
                region.effect.targetKey = node["effect"]["target_key"].as<std::string>();
            }

            this->regions.push_back(region);
            std::cout << "[RaceControl] Loaded region: " << region.name
                      << " (" << region.points.size() << " points)" << std::endl;
        }
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "[RaceControl] YAML parsing error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RaceControl] Error: " << e.what() << std::endl;
    }
}
void RaceControl::applyMemoryEffect(const RegionEffect &effect)
{
    QSharedMemory targetMem;

    if (effect.targetKey == "home") {
        targetMem.setKey(QString::fromStdString("remain_hp"));
        std::cout << "home" << std::endl;
        if (!targetMem.attach())
            return;
        targetMem.lock();
        int *data = reinterpret_cast<int *>(targetMem.data());
        *data += 50;
        targetMem.unlock();
        targetMem.detach();
        updateBullet(this->bulletablesend, 1);
    }
    
}

void RaceControl::onTimerTimeout()
{
    int currentValue = 0;
    if (timeMemory.isAttached())
    {
        timeMemory.lock();
        currentValue = *reinterpret_cast<int *>(timeMemory.data());
        timeMemory.unlock();
    }

    if (currentValue > 0)
    {
        OnSlider(currentValue - 1);
        try
        {
            // map -> base_link
            geometry_msgs::msg::TransformStamped tf;
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            curr_pos_.x = tf.transform.translation.x;
            curr_pos_.y = tf.transform.translation.y;
            curr_pos_.z = tf.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            // 忽略 TF 错误 (比如刚启动时 TF 树还没建立)
            // RCLCPP_WARN(ros_node_->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        // 4. 遍历区域判定
        for (const auto &region : regions)
        {
            if (!region.active)
                continue;

            if (is_in_region(curr_pos_, region))
            {
                std::cout << "[RaceControl] Triggered: " << region.name << std::endl;
                applyMemoryEffect(region.effect);
            }
        }
    }
    else
    {
        timer->stop();
    }
}

// Register plugin
GZ_ADD_PLUGIN(RaceControl, gz::gui::Plugin);
