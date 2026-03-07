#include <cmath>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_door/proximity_door_plugin.hpp>
#include <sdf/sdf.hh>

namespace gazebo {

class ProximityDoorPlugin::Impl {
public:
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  event::ConnectionPtr update_connection_;

  std::string joint_name_;
  std::string target_model_name_;
  double trigger_distance_{1.5};
  double open_angle_{1.2};
  double closed_angle_{0.0};
  double open_speed_{1.0};
  double close_speed_{0.7};
  double hold_open_time_{1.0};
  double doorway_x_{0.0};
  double doorway_y_{0.0};
  common::Time last_open_trigger_time_;

  void OnUpdate() {
    if (!this->joint_ || !this->world_)
    {
      return;
    }

    auto target = this->world_->ModelByName(this->target_model_name_);
    bool should_open = false;

    if (target)
    {
      const auto target_pose = target->WorldPose();
      const double dx = target_pose.Pos().X() - this->doorway_x_;
      const double dy = target_pose.Pos().Y() - this->doorway_y_;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist <= this->trigger_distance_)
      {
        should_open = true;
        this->last_open_trigger_time_ = this->world_->SimTime();
      }
    }

    if (!should_open)
    {
      const double elapsed = (this->world_->SimTime() - this->last_open_trigger_time_).Double();
      should_open = elapsed < this->hold_open_time_;
    }

    const double current = this->joint_->Position(0);
    const double target_angle = should_open ? this->open_angle_ : this->closed_angle_;
    const double speed = should_open ? this->open_speed_ : this->close_speed_;
    const double error = target_angle - current;

    // Simple proportional position tracking with velocity limiting.
    double cmd = 4.0 * error;
    if (cmd > speed)
      cmd = speed;
    if (cmd < -speed)
      cmd = -speed;

    this->joint_->SetParam("fmax", 0, 50.0);
    this->joint_->SetParam("vel", 0, cmd);
  }
};

ProximityDoorPlugin::ProximityDoorPlugin() : impl_(std::make_unique<Impl>()) {}

ProximityDoorPlugin::~ProximityDoorPlugin() = default;

void ProximityDoorPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  impl_->model_ = model;
  impl_->world_ = model->GetWorld();

  impl_->joint_name_ = sdf->HasElement("joint_name") ? sdf->Get<std::string>("joint_name") : "door_hinge";
  impl_->target_model_name_ = sdf->HasElement("target_model") ? sdf->Get<std::string>("target_model") : "actor";
  impl_->trigger_distance_ = sdf->HasElement("trigger_distance") ? sdf->Get<double>("trigger_distance") : 1.5;
  impl_->open_angle_ = sdf->HasElement("open_angle") ? sdf->Get<double>("open_angle") : 1.2;
  impl_->closed_angle_ = sdf->HasElement("closed_angle") ? sdf->Get<double>("closed_angle") : 0.0;
  impl_->open_speed_ = sdf->HasElement("open_speed") ? sdf->Get<double>("open_speed") : 1.0;
  impl_->close_speed_ = sdf->HasElement("close_speed") ? sdf->Get<double>("close_speed") : 0.7;
  impl_->hold_open_time_ = sdf->HasElement("hold_open_time") ? sdf->Get<double>("hold_open_time") : 1.0;
  impl_->doorway_x_ = sdf->HasElement("doorway_x") ? sdf->Get<double>("doorway_x") : 0.0;
  impl_->doorway_y_ = sdf->HasElement("doorway_y") ? sdf->Get<double>("doorway_y") : 0.0;

  impl_->joint_ = impl_->model_->GetJoint(impl_->joint_name_);
  if (!impl_->joint_) {
      gzerr << "[ProximityDoorPlugin] Joint '" << impl_->joint_name_ << "' not found in model '"
            << impl_->model_->GetName() << "'.\n";
      return;
  }

  impl_->last_open_trigger_time_ = impl_->world_->SimTime();
  impl_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&Impl::OnUpdate, impl_.get()));
  
    gzmsg << "[ProximityDoorPlugin] Loaded for model '" << impl_->model_->GetName()
          << "', watching target model '" << impl_->target_model_name_ << "'.\n";
}

GZ_REGISTER_MODEL_PLUGIN(ProximityDoorPlugin)

}  // namespace gazebo
