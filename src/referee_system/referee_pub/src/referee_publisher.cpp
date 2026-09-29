#include "referee_pub/referee_publisher.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <cstring>

#include <QSharedMemory>
#include <gz/gui/qt.h>

using namespace std::chrono_literals;

std::map<std::string, int> referee_dict{
    {"remain_hp", 400},
    {"max_hp", 400},
    {"bullet_remaining_num_17mm", 300},
    {"bullet_cooling_speed", 0},
    {"shooter_heat_limit", 0},
    {"shooter_heat_now", 0},
    {"remain_energy", 0},
    {"health_state", 0},
    {"state_now", 0},
    {"stage_remain_time", 420},
    {"game_progress", 4},
    {"ally_outpost_hp", 1500},
    {"ally_base_hp", 5000},
    {"rfid_status", 0},
    {"event_data", 0},
    {"defend_fortress", 0},
    {"enemy_outpost_alive", 1},
    {"catch_hero", 0},
    {"catch_engineer", 0},
    {"rush_home", 0},
    {"bumpy_exist_enemy", 0},
    {"enemy_base_flower", 0},
    {"could_fire", 0},
    {"head_status", 2}

};

std::vector<std::string> referee_keys{
    "remain_hp",
    "max_hp",
    "bullet_remaining_num_17mm",
    "bullet_cooling_speed",
    "shooter_heat_limit",
    "shooter_heat_now",
    "remain_energy",
    "health_state",
    "state_now",
    "stage_remain_time",
    "game_progress",
    "ally_outpost_hp",
    "ally_base_hp",
    "rfid_status",
    "event_data",
    "defend_fortress",
    "enemy_outpost_alive",
    "catch_hero",
    "catch_engineer",
    "rush_home",
    "bumpy_exist_enemy",
    "enemy_base_flower",
    "could_fire",
    "head_status"
};

referee_publisher::referee_publisher()
    : Node("referee_publisher"), count_(0)
{
    for (const auto & key : referee_keys)
    {
        this->declare_parameter<int>(key, referee_dict[key]);
        this->get_parameter(key, referee_dict[key]);
    }

    publisher_ = this->create_publisher<referee_msg::msg::Referee>("Referee", 10);

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&referee_publisher::timer_callback, this)
    );
}

referee_publisher::~referee_publisher()
{
    if (shared_memory.isAttached())
    {
        shared_memory.detach();
    }
}

void referee_publisher::memory(std::string msg_name)
{
    shared_memory.setKey(QString::fromStdString(msg_name));

    if (!shared_memory.attach())
    {
        return;
    }

    if (!shared_memory.constData())
    {
        shared_memory.detach();
        return;
    }

    shared_memory.lock();
    memcpy(&referee_dict[msg_name], shared_memory.constData(), sizeof(int));
    shared_memory.unlock();

    shared_memory.detach();
}

void referee_publisher::timer_callback()
{
    for (const auto & key : referee_keys)
    {
        referee_publisher::memory(key);
    }

    auto message = referee_msg::msg::Referee();

    message.remain_hp = static_cast<uint16_t>(referee_dict["remain_hp"]);
    message.max_hp = static_cast<uint16_t>(referee_dict["max_hp"]);
    message.bullet_remaining_num_17mm = static_cast<uint16_t>(referee_dict["bullet_remaining_num_17mm"]);
    message.bullet_cooling_speed = static_cast<uint16_t>(referee_dict["bullet_cooling_speed"]);
    message.shooter_heat_limit = static_cast<uint16_t>(referee_dict["shooter_heat_limit"]);
    message.shooter_heat_now = static_cast<uint16_t>(referee_dict["shooter_heat_now"]);

    message.remain_energy = static_cast<uint8_t>(referee_dict["remain_energy"]);
    message.health_state = static_cast<uint8_t>(referee_dict["health_state"]);
    message.state_now = static_cast<uint8_t>(referee_dict["state_now"]);

    message.stage_remain_time = static_cast<uint16_t>(referee_dict["stage_remain_time"]);
    message.game_progress = static_cast<uint8_t>(referee_dict["game_progress"]);

    message.ally_outpost_hp = static_cast<uint16_t>(referee_dict["ally_outpost_hp"]);
    message.ally_base_hp = static_cast<uint16_t>(referee_dict["ally_base_hp"]);

    message.rfid_status = static_cast<uint32_t>(referee_dict["rfid_status"]);
    message.event_data = static_cast<uint32_t>(referee_dict["event_data"]);

    message.defend_fortress = static_cast<uint8_t>(referee_dict["defend_fortress"]);
    message.enemy_outpost_alive = static_cast<uint8_t>(referee_dict["enemy_outpost_alive"]);
    message.catch_hero = static_cast<uint8_t>(referee_dict["catch_hero"]);
    message.catch_engineer = static_cast<uint8_t>(referee_dict["catch_engineer"]);
    message.rush_home = static_cast<uint8_t>(referee_dict["rush_home"]);
    message.bumpy_exist_enemy = static_cast<uint8_t>(referee_dict["bumpy_exist_enemy"]);
    message.enemy_base_flower = static_cast<uint8_t>(referee_dict["enemy_base_flower"]);
    message.could_fire = static_cast<uint8_t>(referee_dict["could_fire"]);
    message.head_status = static_cast<uint8_t>(referee_dict["head_status"]);

    publisher_->publish(message);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<referee_publisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}