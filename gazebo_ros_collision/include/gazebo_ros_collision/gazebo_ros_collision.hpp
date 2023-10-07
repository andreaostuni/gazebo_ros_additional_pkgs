// Copyright 2023 Andrea Ostuni - PIC4SeR
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef GAZEBO_ROS_COLLISION_PLUGIN_HPP
#define GAZEBO_ROS_COLLISION_PLUGIN_HPP

#include <string>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
// #include <gazebo_msgs/msg/contact_state.hpp>
// #include <gazebo_msgs/msg/contacts_state.hpp>
// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
// #include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <gazebo_collision_msgs/msg/collision.hpp>

#include <memory>
#include <string>

namespace gazebo_ros_collision
{

class CollisionPluginPrivate;

class CollisionPlugin : public gazebo::SensorPlugin
{

public:
  /**
   * @brief Constructor for the CollisionPlugin class.
   */
  CollisionPlugin();

  /**
   * @brief Destructor for the CollisionPlugin class.
   */
  virtual ~CollisionPlugin();

  /**
   * @brief Load the plugin.
   * @param _sensor Pointer to the sensor that loaded this plugin.
   * @param _sdf SDF element that describes the plugin.
   */
  virtual void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  /**
   * @brief Pointer to the implementation.
   */
  std::unique_ptr<CollisionPluginPrivate> impl_;
};

} // namespace gazebo_ros_collision

#endif // GAZEBO_ROS_COLLISION_PLUGIN_HPP
