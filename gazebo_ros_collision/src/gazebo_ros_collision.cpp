#include "gazebo_ros_collision/gazebo_ros_collision.hpp"

namespace gazebo_ros_collision
{

CollisionPlugin::CollisionPlugin() : gazebo::SensorPlugin()
{
}

CollisionPlugin::~CollisionPlugin()
{
}

void CollisionPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  node_ = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(node_->get_logger(), "CollisionPlugin: Loading plugin.");
  // Get the parent sensor.
  this->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
  if(!this->parent_sensor_){
    RCLCPP_ERROR(node_->get_logger(), "ContactPlugin requires a ContactSensor.");
    node_.reset();
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = this->node_->get_qos();

  // Contact state publisher
  // pub_ = this->node_->create_publisher<gazebo_msgs::msg::ContactsState>(
  //   "bumper_states", qos.get_publisher_qos("bumper_states", rclcpp::SensorDataQoS().reliable()));
  pub_ = this->node_->create_publisher<gazebo_collision_msgs::msg::Collision>(
    "collision", qos.get_publisher_qos("collision", rclcpp::SensorDataQoS().reliable()));

  RCLCPP_INFO(
    node_->get_logger(), "Publishing contact states to [%s]",
    pub_->get_topic_name());
  
  // Get tf frame for output
  frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;
  
  terrain_name_ = _sdf->Get<std::string>("terrain_name", "terrain").first;

  this->update_connection_ = this->parent_sensor_->ConnectUpdated(
    std::bind(&CollisionPlugin::OnUpdate, this));

  this->parent_sensor_->SetActive(true);

}

void CollisionPlugin::OnUpdate()
{
  // jetson_dqn::Contact msg;
  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = this->parentSensor->ParentName();
  // msg.header.frame_id = msg.header.frame_id.substr(msg.header.frame_id.find("::") + 2);
  std::vector<std::string> objs_hit;

  gazebo::msgs::Contacts contacts;
  contacts = parent_sensor_->Contacts();
  gazebo_collision_msgs::msg::Collision msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = node_->now();


  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string obj_name = contacts.contact(i).collision2();

    // Remove the namespace from the object name
    // while (true)
    // {
    //   if (obj_name.find(std::string("::")) != std::string::npos)
    //   {
    //     obj_name = obj_name.substr(obj_name.find("::") + 2);
    //   }
    //   else
    //   {
    //     break;
    //   }
    // }


    // Remove the terrain from the collision list
    // use copy_if() to remove the terrain from the list and lamda function to check if is already in the list

    if (obj_name.find(terrain_name_) != std::string::npos)
    {
      continue;
    }

    // Add the object to the list if it is not already there
    if (std::find(objs_hit.begin(), objs_hit.end(), obj_name) == objs_hit.end())
    {
      objs_hit.push_back(obj_name);
    }
  }

  if (objs_hit.size() > 0)
  {
    msg.objects_hit = objs_hit;
    // auto contact_state_msg = gazebo_ros::Convert<gazebo_msgs::msg::ContactsState>(contacts);
    // contact_state_msg.header.frame_id = frame_name_;
  }
  pub_->publish(msg);  

  return;
}

GZ_REGISTER_SENSOR_PLUGIN(CollisionPlugin)
} // namespace gazebo_ros_collision