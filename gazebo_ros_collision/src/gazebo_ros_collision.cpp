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


#include "gazebo_ros_collision/gazebo_ros_collision.hpp"

namespace gazebo_ros_collision
{

class CollisionPluginPrivate
{
public:
  /**
   * @brief OnUpdate callback that publishes the contact state.
   */
  void OnUpdate();

  /**
   * @brief Pointer to the contact sensor.
   */
  gazebo::sensors::ContactSensorPtr parent_sensor_;

  /**
   * @brief Connection that maintains a link between the contact sensor's
   *  updated signal and the OnUpdate callback.
   */
  gazebo::event::ConnectionPtr update_connection_;

  /**
   * @brief pointer to the GazeboROS node.
   */
  gazebo_ros::Node::SharedPtr node_{nullptr};

  /**
   * @brief Frame name, to be used by TF.
   */
  std::string frame_name_;

  /**
   * @brief Graund name, to be filtered from contact states.
   */
  std::string terrain_name_;

  /**
   * @brief ROS2 publisher for the contact topic.
   */
  rclcpp::Publisher<gazebo_collision_msgs::msg::Collision>::SharedPtr pub_; //gazeboROS
};

CollisionPlugin::CollisionPlugin()
: impl_(std::make_unique<CollisionPluginPrivate>())
{
}

CollisionPlugin::~CollisionPlugin()
{
  impl_->node_.reset();
}

void CollisionPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->node_ = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(impl_->node_->get_logger(), "CollisionPlugin: Loading plugin.");
  // Get the parent sensor.
  impl_->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
  if (!impl_->parent_sensor_) {
    RCLCPP_ERROR(impl_->node_->get_logger(), "ContactPlugin requires a ContactSensor.");
    impl_->node_.reset();
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->node_->get_qos();

  // Contact state publisher
  impl_->pub_ = impl_->node_->create_publisher<gazebo_collision_msgs::msg::Collision>(
    "collision", qos.get_publisher_qos("collision", rclcpp::SensorDataQoS().reliable()));

  RCLCPP_INFO(
    impl_->node_->get_logger(), "Publishing contact states to [%s]",
    impl_->pub_->get_topic_name());

  // Get tf frame for output
  impl_->frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;

  impl_->terrain_name_ = _sdf->Get<std::string>("terrain_name", "terrain").first;

  impl_->update_connection_ = impl_->parent_sensor_->ConnectUpdated(
    std::bind(&CollisionPluginPrivate::OnUpdate, impl_.get()));

  impl_->parent_sensor_->SetActive(true);

}

void CollisionPluginPrivate::OnUpdate()
{
  std::vector<std::string> objs_hit;

  gazebo::msgs::Contacts contacts;
  contacts = parent_sensor_->Contacts();
  gazebo_collision_msgs::msg::Collision msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = node_->now();

  // Remove the terrain from the collision list
  std::for_each(
    contacts.contact().begin(), contacts.contact().end(),
    [&](gazebo::msgs::Contact contact) {
      if (contact.collision2().find(terrain_name_) == std::string::npos) {
        objs_hit.push_back(contact.collision2());
      }
    });


  if (!objs_hit.empty()) {
    // insert the collision list into the message and publish it
    std::unique_copy(objs_hit.begin(), objs_hit.end(), std::back_inserter(msg.objects_hit));
    pub_->publish(msg);
  }

  return;
}

GZ_REGISTER_SENSOR_PLUGIN(CollisionPlugin)
} // namespace gazebo_ros_collision
