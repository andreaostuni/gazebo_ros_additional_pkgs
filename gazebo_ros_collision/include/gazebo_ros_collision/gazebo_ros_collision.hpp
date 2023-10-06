#ifndef GAZEBO_ROS_COLLISION_PLUGIN_HPP
#define GAZEBO_ROS_COLLISION_PLUGIN_HPP

#include <string>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <gazebo_collision_msgs/msg/collision.hpp>

#include <memory>
#include <string>

namespace gazebo_ros_collision
{

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
   * @brief Callback that receives the contact sensor's update signal.
   */
  virtual void OnUpdate();

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
  
  // rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr pub_; //gazeboROS
  rclcpp::Publisher<gazebo_collision_msgs::msg::Collision>::SharedPtr pub_; //gazeboROS
};

} // namespace gazebo_ros_collision

#endif // GAZEBO_ROS_COLLISION_PLUGIN_HPP