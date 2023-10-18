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


#ifndef GAZEBO_ROS_POSE3D_PLUGIN_HPP
#define GAZEBO_ROS_POSE3D_PLUGIN_HPP

#include <string>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// ROS
#include <nav_msgs/msg/odometry.hpp>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace gazebo_ros_pose3d
{

class GazeboPose3DPrivate;

/// Broadcasts the inertial pose of an model's link via a nav_msgs/Odometry message on a ROS topic.
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_pose3d" filename="libgazebo_ros_pose3d.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>odom:=p3d_demo</remapping>

      </ros>

      <!-- Name of the link within this model whose pose will be published -->
      <body_name>box_link</body_name>

      <!-- Name of another link within this model to use as a reference frame.
           Remove the tag to use the world as a reference. -->
      <frame_name>sphere_link</frame_name>

      <!-- Publish the pose as a tf2 transform. -->
      <publish_tf>true</publish_tf>

      <!-- Update rate in Hz, defaults to 0.0, which means as fast as possible -->
      <update_rate>1</update_rate>

      <!-- Translation offset to be added to the pose. -->
      <xyz_offset>10 10 10</xyz_offset>

      <!-- Rotation offset to be added to the pose, in Euler angles. -->
      <rpy_offset>0.1 0.1 0.1</rpy_offset>

      <!-- Standard deviation of the noise to be added to the reported velocities. -->
      <gaussian_noise>0.01</gaussian_noise>

    </plugin>
  \endcode
*/

class GazeboPose3D : public gazebo::ModelPlugin
{

public:
  /**
   * @brief Constructor for the GazeboPose3DPlugin class.
   */
  GazeboPose3D();

  /**
   * @brief Destructor for the GazeboPose3D class.
   */
  virtual ~GazeboPose3D();

  /**
   * @brief Load the plugin.
   * @param _sensor Pointer to the sensor that loaded this plugin.
   * @param _sdf SDF element that describes the plugin.
   */
  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  /**
   * @brief Pointer to the implementation.
   */
  std::unique_ptr<GazeboPose3DPrivate> impl_;
};

} // namespace gazebo_ros_pose3d

#endif // GAZEBO_ROS_POSE3D_PLUGIN_HPP
