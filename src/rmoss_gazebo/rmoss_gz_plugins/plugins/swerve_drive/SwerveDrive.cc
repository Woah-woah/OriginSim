#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
namespace systems
{
class SwerveDrive : public System, public ISystemConfigure,
                    public ISystemPreUpdate, public ISystemPostUpdate
{
public:
  void Configure(const Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      EntityComponentManager &ecm, EventManager &) override
  {
    Model model(entity);
    this->chassis = Link(model.LinkByName(ecm, sdf->Get<std::string>("chassis_link")));
    this->radius = sdf->Get<double>("wheel_radius", 0.0758).first;
    this->wheelbase = sdf->Get<double>("wheelbase", this->wheelbase).first;
    this->track = sdf->Get<double>("track_width", this->track).first;
    this->timeout = sdf->Get<double>("command_timeout", 0.5).first;
    if (!this->chassis.Valid(ecm) || !std::isfinite(this->radius) ||
        !std::isfinite(this->wheelbase) || !std::isfinite(this->track) ||
        !std::isfinite(this->timeout) || this->radius <= 0 ||
        this->wheelbase <= 0 || this->track <= 0 || this->timeout <= 0)
    {
      gzerr << "SwerveDrive: invalid chassis or dimensions/timeout\n";
      return;
    }
    const std::array<std::string, 4> prefixes = {{
      "front_left", "front_right", "rear_left", "rear_right"}};
    for (size_t i = 0; i < prefixes.size(); ++i)
    {
      this->wheels[i] = model.JointByName(ecm,
          sdf->Get<std::string>(prefixes[i] + "_joint"));
      this->steering[i] = model.JointByName(ecm,
          sdf->Get<std::string>(prefixes[i] + "_steering_joint",
          prefixes[i] + "_steering_joint").first);
      if (this->wheels[i] == kNullEntity || this->steering[i] == kNullEntity)
      {
        gzerr << "SwerveDrive: missing joints for " << prefixes[i] << "\n";
        return;
      }
      if (!ecm.Component<components::JointPosition>(this->steering[i]))
        ecm.CreateComponent(this->steering[i], components::JointPosition());
    }
    this->chassis.EnableVelocityChecks(ecm);
    this->odomPub = this->node.Advertise<msgs::Odometry>(
        sdf->Get<std::string>("odom_topic", "/odometry").first);
    this->childFrame = "/" + sdf->Get<std::string>("chassis_link");
    const auto topic = sdf->Get<std::string>("topic", "/cmd_vel").first;
    this->ready = this->node.Subscribe(topic, &SwerveDrive::OnCommand, this);
    gzmsg << "SwerveDrive subscribing to " << topic << "\n";
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (!this->ready || info.paused)
      return;
    msgs::Twist command;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // Timestamp arrivals in simulation time so slow and paused runs behave alike.
      if (this->pending)
      {
        this->lastCommand = info.simTime;
        this->pending = false;
      }
      if (info.dt < std::chrono::steady_clock::duration::zero())
        this->target.Clear();
      if (info.simTime >= this->lastCommand &&
          std::chrono::duration<double>(info.simTime - this->lastCommand).count()
          <= this->timeout)
        command = this->target;
    }
    const double pi = std::acos(-1.0);
    for (size_t i = 0; i < this->wheels.size(); ++i)
    {
      const auto *position = ecm.Component<components::JointPosition>(this->steering[i]);
      if (!position || position->Data().empty())
        continue;
      const double angle = position->Data()[0];
      const double x = (i < 2 ? 0.5 : -0.5) * this->wheelbase;
      const double y = (i % 2 == 0 ? 0.5 : -0.5) * this->track;
      const double vx = command.linear().x() - y * command.angular().z();
      const double vy = command.linear().y() + x * command.angular().z();
      double speed = std::hypot(vx, vy) / this->radius;
      double error = 0;
      if (speed > 1e-6)
      {
        error = std::remainder(std::atan2(vy, vx) - angle, 2 * pi);
        // Equivalent heading plus reverse drive avoids steering through 180 degrees.
        if (std::abs(error) > pi / 2)
        {
          error = std::remainder(error + pi, 2 * pi);
          speed = -speed;
        }
        this->headings[i] = angle + error;
      }
      else
      {
        error = std::remainder(this->headings[i] - angle, 2 * pi);
      }
      SetVelocity(ecm, this->steering[i], std::max(-8.0, std::min(8.0, 12 * error)));
      // Project onto the actual rolling direction while steering converges.
      SetVelocity(ecm, this->wheels[i], speed * std::cos(error));
    }
  }

  void PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm) override
  {
    if (!this->ready || info.paused)
      return;
    const auto pose = this->chassis.WorldPose(ecm);
    const auto linear = this->chassis.WorldLinearVelocity(ecm);
    const auto angular = this->chassis.WorldAngularVelocity(ecm);
    if (!pose || !linear || !angular)
      return;
    msgs::Odometry msg;
    msgs::Set(msg.mutable_pose(), *pose);
    // Odometry twist is expressed in child_frame_id, not the world frame.
    msgs::Set(msg.mutable_twist()->mutable_linear(), pose->Rot().RotateVectorReverse(*linear));
    msgs::Set(msg.mutable_twist()->mutable_angular(), pose->Rot().RotateVectorReverse(*angular));
    *msg.mutable_header()->mutable_stamp() = convert<msgs::Time>(info.simTime);
    auto *frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("/odom");
    auto *child = msg.mutable_header()->add_data();
    child->set_key("child_frame_id");
    child->add_value(this->childFrame);
    this->odomPub.Publish(msg);
  }

private:
  static void SetVelocity(EntityComponentManager &ecm, Entity joint, double value)
  {
    auto *cmd = ecm.Component<components::JointVelocityCmd>(joint);
    if (cmd)
      cmd->Data() = {value};
    else
      ecm.CreateComponent(joint, components::JointVelocityCmd(std::vector<double>{value}));
  }

  void OnCommand(const msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->target.Clear();
    if (std::isfinite(msg.linear().x()) && std::isfinite(msg.linear().y()) &&
        std::isfinite(msg.angular().z()))
      this->target = msg;
    this->pending = true;
  }

  transport::Node node;
  transport::Node::Publisher odomPub;
  Link chassis{kNullEntity};
  std::array<Entity, 4> wheels{};
  std::array<Entity, 4> steering{};
  std::array<double, 4> headings{};
  double radius{0.0758}, wheelbase{0.2465120200145953}, track{0.2465120200145953}, timeout{0.5};
  bool ready{false}, pending{false};
  std::string childFrame;
  std::mutex mutex;
  msgs::Twist target;
  std::chrono::steady_clock::duration lastCommand{};
};
}
}
}

GZ_ADD_PLUGIN(gz::sim::systems::SwerveDrive, gz::sim::System,
    gz::sim::systems::SwerveDrive::ISystemConfigure,
    gz::sim::systems::SwerveDrive::ISystemPreUpdate,
    gz::sim::systems::SwerveDrive::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SwerveDrive, "gz::sim::systems::SwerveDrive")
