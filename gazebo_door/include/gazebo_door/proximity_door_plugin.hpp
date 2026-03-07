#ifndef GAZEBO_DOOR_PROXIMITY_DOOR_PLUGIN_HPP
#define GAZEBO_DOOR_PROXIMITY_DOOR_PLUGIN_HPP

#include <cmath>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo {


class ProximityDoorPlugin : public ModelPlugin {
public:
  ProximityDoorPlugin();
  ~ProximityDoorPlugin() override;
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gazebo

#endif  // GAZEBO_DOOR_PROXIMITY_DOOR_PLUGIN_HPP