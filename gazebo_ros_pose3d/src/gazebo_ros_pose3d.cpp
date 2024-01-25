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


#include "gazebo_ros_pose3d/gazebo_ros_pose3d.hpp"

namespace gazebo_ros_pose3d
{

class GazeboPose3DPrivate
{
public:
  /**
   * @brief Callback to be called at every simulation iteration
   * @param[in] info Updated simulation info
   */
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /**
   * @brief The link being tracked.
  */
  gazebo::physics::LinkPtr link_{nullptr};

  /**
   * @brief The body of the frame to display pose, twist
  */
  gazebo::physics::LinkPtr reference_link_{nullptr};

  /**
   * @brief Connection that maintains a link between the contact sensor's
   *  updated signal and the OnUpdate callback.
   */
  gazebo::event::ConnectionPtr update_connection_;

  /**
   * @brief pointer to the GazeboROS node.
   */
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /**
   * @brief Odom topic name
   */
  std::string topic_name_{"odom"};

    /**
   * @brief Frame transform name, should match name of reference link, or be world.
   */
  std::string frame_name_{"world"};

  /**
   * @brief Constant xyz and rpy offsets
   */
  ignition::math::Pose3d offset_;

  /**
   * @brief Keep track of the last update time.
   */
  gazebo::common::Time last_time_;

  /**
   * @brief  Publish rate in Hz.
   */
  double update_rate_{0.0};

  /**
   * @brief Gaussian noise to be added to the reported velocities.
   */
  double gaussian_noise_;

  /**
   * @brief Whether to publish the pose as a tf2 transform.
   */
  bool publish_tf_{false};

  /**
   * @brief Whether to publish the velocity as a relative velocity.
   */
  bool relative_velocity_{false};

  /**
   * @brief Whether to publish a transform from map to world.
   */
  bool publish_map_to_world_{false};

  /**
   * @brief ROS2 publisher for odometry messages.
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  /**
  * @brief To broadcast TFs
  */
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

GazeboPose3D::GazeboPose3D()
: impl_(std::make_unique<GazeboPose3DPrivate>())
{
}

GazeboPose3D::~GazeboPose3D()
{
  impl_->ros_node_.reset();
}

void GazeboPose3D::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Configure the plugin from the SDF file
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "pose3d plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }

  std::string link_name;
  if (!sdf->HasElement("body_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <body_name>, cannot proceed");
    return;
  } else {
    link_name = sdf->GetElement("body_name")->Get<std::string>();
  }

  impl_->link_ = model->GetLink(link_name);
  if (!impl_->link_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "body_name: %s does not exist\n",
      link_name.c_str());
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    impl_->topic_name_, qos.get_publisher_qos(
      impl_->topic_name_, rclcpp::SensorDataQoS().reliable()));
  impl_->topic_name_ = impl_->pub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Publishing on topic [%s]", impl_->topic_name_.c_str());

  if (sdf->HasElement("xyz_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<xyz_offsets> is deprecated, use <xyz_offset> instead.");
    impl_->offset_.Pos() = sdf->GetElement("xyz_offsets")->Get<ignition::math::Vector3d>();
  }
  if (!sdf->HasElement("xyz_offset")) {
    if (!sdf->HasElement("xyz_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <xyz_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Pos() = sdf->GetElement("xyz_offset")->Get<ignition::math::Vector3d>();
  }

  if (sdf->HasElement("rpy_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<rpy_offsets> is deprecated, use <rpy_offset> instead.");
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offsets")->Get<ignition::math::Vector3d>());
  }
  if (!sdf->HasElement("rpy_offset")) {
    if (!sdf->HasElement("rpy_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <rpy_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offset")->Get<ignition::math::Vector3d>());
  }

  if (!sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <gassian_noise>, defaults to 0.0");
    impl_->gaussian_noise_ = 0;
  } else {
    impl_->gaussian_noise_ = sdf->GetElement("gaussian_noise")->Get<double>();
  }

  impl_->last_time_ = model->GetWorld()->SimTime();

  if (!sdf->HasElement("frame_name")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <frame_name>, defaults to world");
  } else {
    impl_->frame_name_ = sdf->GetElement("frame_name")->Get<std::string>();
  }

  // If frame_name specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (impl_->frame_name_ != "/world" && impl_->frame_name_ != "world" &&
    impl_->frame_name_ != "/map" && impl_->frame_name_ != "map")
  {
    impl_->reference_link_ = model->GetLink(impl_->frame_name_);
    if (!impl_->reference_link_) {
      RCLCPP_WARN(
        impl_->ros_node_->get_logger(), "<frame_name> [%s] does not exist.",
        impl_->frame_name_.c_str());
    }
  }

  if(!sdf->HasElement("publish_tf")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <publish_tf>, defaults to false");
  } else {
    impl_->publish_tf_ = sdf->GetElement("publish_tf")->Get<bool>();
  }

  if(!sdf->HasElement("relative_velocity")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <relative_velocity>, defaults to false");
  } else {
    impl_->relative_velocity_ = sdf->GetElement("relative_velocity")->Get<bool>();
  }

  if(!sdf->HasElement("publish_map_to_world")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <publish_map_to_world>, defaults to false");
  } else {
    impl_->publish_map_to_world_ = sdf->GetElement("publish_map_to_world")->Get<bool>();
  }

  if(impl_->publish_tf_)
  {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing TFs between [%s] and [%s]",
      impl_->frame_name_.c_str(), link_name.c_str());
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
  } 

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboPose3DPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

// Update the controller
void GazeboPose3DPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (!link_) {
    return;
  }

  gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 &&
    (current_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }

  // If we don't have any subscribers, don't bother composing and sending the message
  if (ros_node_->count_subscribers(topic_name_) == 0 && !publish_tf_) {
    return;
  }

  // Differentiate to get accelerations
  double tmp_dt = current_time.Double() - last_time_.Double();
  if (tmp_dt == 0) {
    return;
  }
  nav_msgs::msg::Odometry pose_msg;

  // Copy data into pose message
  pose_msg.header.frame_id = frame_name_;
  pose_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  pose_msg.child_frame_id = link_->GetName();

  // Get inertial rates
  ignition::math::Vector3d vpos, veul;
  if (relative_velocity_) {
    vpos = link_->RelativeLinearVel();
    veul = link_->RelativeAngularVel();
  } else {
    vpos = link_->WorldLinearVel();
    veul = link_->WorldAngularVel();
  }

  // Get pose/orientation
  auto pose = link_->WorldPose();

  // Apply reference frame
  if (reference_link_) {
    // Convert to relative pose, rates
    auto frame_pose = reference_link_->WorldPose();
    auto frame_vpos = reference_link_->WorldLinearVel();
    auto frame_veul = reference_link_->WorldAngularVel();

    pose.Pos() = pose.Pos() - frame_pose.Pos();
    pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
    pose.Rot() *= frame_pose.Rot().Inverse();

    if (!relative_velocity_){
      vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
      veul = frame_pose.Rot().RotateVector(veul - frame_veul);
    } 
  }

  // Apply constant offsets

  // Apply XYZ offsets and get position and rotation components
  pose.Pos() = pose.Pos() + offset_.Pos();
  // Apply RPY offsets
  pose.Rot() = offset_.Rot() * pose.Rot();
  pose.Rot().Normalize();

  // Fill out messages
  pose_msg.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  pose_msg.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  pose_msg.twist.twist.linear.x = vpos.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.linear.y = vpos.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.linear.z = vpos.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.x = veul.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.y = veul.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.z = veul.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);

  // Fill in covariance matrix
  /// @TODO: let user set separate linear and angular covariance values
  double gn2 = gaussian_noise_ * gaussian_noise_;
  pose_msg.pose.covariance[0] = gn2;
  pose_msg.pose.covariance[7] = gn2;
  pose_msg.pose.covariance[14] = gn2;
  pose_msg.pose.covariance[21] = gn2;
  pose_msg.pose.covariance[28] = gn2;
  pose_msg.pose.covariance[35] = gn2;
  pose_msg.twist.covariance[0] = gn2;
  pose_msg.twist.covariance[7] = gn2;
  pose_msg.twist.covariance[14] = gn2;
  pose_msg.twist.covariance[21] = gn2;
  pose_msg.twist.covariance[28] = gn2;
  pose_msg.twist.covariance[35] = gn2;

  if(publish_tf_)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = pose_msg.header.stamp;
    msg.header.frame_id = frame_name_;
    msg.child_frame_id = link_->GetName();
    msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_msg.pose.pose.position);
    msg.transform.rotation = pose_msg.pose.pose.orientation;

    transform_broadcaster_->sendTransform(msg);
  }

  if(publish_map_to_world_)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = pose_msg.header.stamp;
    msg.header.frame_id = frame_name_;
    msg.child_frame_id = "map";
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(gazebo_ros::Convert<geometry_msgs::msg::Point>(offset_.Pos()));
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(offset_.Rot());
    transform_broadcaster_->sendTransform(msg);
  }

  // Publish to ROS
  pub_->publish(pose_msg);
  // Save last time stamp
  last_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPose3D)
} // namespace gazebo_ros_pose3d
