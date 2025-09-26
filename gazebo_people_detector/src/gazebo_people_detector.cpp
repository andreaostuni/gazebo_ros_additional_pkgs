#include <stdio.h>
#include <string>
#include <set>
#include <deque>

#include <gazebo/physics/physics.hh>
#include "gazebo/sensors/SensorFactory.hh"
#include <gazebo_people_detector/gazebo_people_detector.hpp>
#include "geometry_msgs/msg/point.hpp"

namespace gazebo_people_detector
{

using namespace std::chrono_literals;

// Structure to hold position and timestamp data
struct PositionHistory
{
  ignition::math::Vector3d position;
  gazebo::common::Time timestamp;
};

class PeopleDetectorPrivate
{
public:
  /// Callback when the sensor is updated.
  void OnUpdate();
  /// Callback to get the models list from the parameter server
  void GetAgentsList();

  // Helpers
  bool allowed(const std::string& name) const;
  // Get the model name without suffix
  std::string getModelName(const std::string& name) const;
  // Compute the velocity of a model based on position differences over time
  geometry_msgs::msg::Point ComputeVelocity(const std::string& name, const ignition::math::Vector3d& current_position,
                                            gazebo::common::Time current_time);

  /// \brief ros node required to call the service
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief Pointer to the parent sensor.
  gazebo::sensors::LogicalCameraSensorPtr parent_sensor_;

  /// \brief Frame name, to be used by TF.
  std::string frame_name_;

  /// \brief Pointer to the world.
  gazebo::physics::WorldPtr world_;
  /// \brief Pointer to the physics engine.
  gazebo::physics::PhysicsEnginePtr phys_;
  /// \brief Pointer to a ray shape, to be used for ray queries.
  gazebo::physics::RayShapePtr ray_;  // reuse one ray per update

  /// \brief Connection that maintains a link between the sensor's
  /// updated signal and the OnUpdate callback.
  gazebo::event::ConnectionPtr update_connection_;

  /// \brief Publisher for detected people
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr pub_people_;

  // List of agents names to be detected
  std::vector<std::string> agents_names_;
  // Time of the last update.
  gazebo::common::Time lastUpdate;

  // map of model names to their position history (last n positions)
  std::map<std::string, std::deque<PositionHistory>> position_history_;
  // map of model names to their current velocity estimate (EMA)
  std::map<std::string, geometry_msgs::msg::Point> velocity_estimates_;
  // Maximum number of positions to keep in history for velocity computation
  static const size_t MAX_HISTORY_SIZE = 5;
  // Number of steps to use for velocity computation (positions n steps apart)
  // Higher values provide smoother estimates but slower response to changes
  static const size_t VELOCITY_STEP_SIZE = 3;
  // Exponential moving average smoothing factor (0 < alpha <= 1)
  // Higher values respond faster to changes, lower values provide more smoothing
  static constexpr double EMA_ALPHA = 0.5;
  // Minimum time interval between position updates (in seconds)
  // This prevents storing redundant data when sensor updates very frequently
  static constexpr double MIN_UPDATE_INTERVAL = 0.1;

  // map of model names to their last position update time
  std::map<std::string, gazebo::common::Time> last_position_update_time_;

  // Detected people
  people_msgs::msg::People people;
};

PeopleDetector::PeopleDetector() : impl_(std::make_unique<PeopleDetectorPrivate>())
{
}

PeopleDetector::~PeopleDetector()
{
}

void PeopleDetector::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "PeopleDetector: Loading plugin.");

  // Get the parent sensor.
  impl_->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  if (!impl_->parent_sensor_)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "PeopleDetector requires a LogicalCameraSensor.");
    impl_->ros_node_.reset();
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();

  // People Detector publisher
  impl_->pub_people_ = impl_->ros_node_->create_publisher<people_msgs::msg::People>(
      "detected_people", qos.get_publisher_qos("detected_people", rclcpp::SensorDataQoS().reliable()));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing detected people to [%s]",
              impl_->pub_people_->get_topic_name());

  // Get the tf frame for output
  impl_->frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;

  // Get a pointer to the world.
  impl_->world_ = gazebo::physics::get_world(impl_->parent_sensor_->WorldName());
  if (!impl_->world_)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "PeopleDetector: Failed to get world pointer.");
    impl_->ros_node_.reset();
    return;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "PeopleDetector: Got world pointer.");
  // Get a pointer to the physics engine.
  impl_->phys_ = impl_->world_->Physics();
  if (!impl_->phys_)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "PeopleDetector: Failed to get physics engine pointer.");
    impl_->ros_node_.reset();
    return;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "PeopleDetector: Got physics engine pointer.");
  // Create a ray shape for ray queries
  impl_->ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      impl_->phys_->CreateShape("ray", gazebo::physics::CollisionPtr()));
  if (!impl_->ray_)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "PeopleDetector: Failed to create ray shape.");
    impl_->ros_node_.reset();
    return;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "PeopleDetector: Created ray shape.");

  impl_->update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&PeopleDetectorPrivate::OnUpdate, impl_.get()));

  // Get the agents list from the parameter server
  impl_->GetAgentsList();
}

void PeopleDetectorPrivate::OnUpdate()
{
  // Read latest logical camera image (poses of models relative to camera frame)
  const gazebo::msgs::LogicalCameraImage& img = parent_sensor_->Image();

  people_msgs::msg::People output_msg;
  output_msg.header.stamp = ros_node_->now();
  output_msg.header.frame_id = frame_name_;

  // camera pose in world coordinates
  ignition::math::Pose3d cameraPose = gazebo::msgs::ConvertIgn(img.pose());

  // clear the position history and velocity estimates if the sensor was reset
  if (parent_sensor_->LastUpdateTime() < lastUpdate)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Sensor reset detected, clearing position history and velocity estimates");
    position_history_.clear();
    velocity_estimates_.clear();
    last_position_update_time_.clear();
  }

  // Collect currently detected models to clean up last_positions_ later
  std::set<std::string> detected_models;

  for (int i = 0; i < img.model_size(); ++i)
  {
    const std::string name = img.model(i).name();
    // we need to filter the models, as the logical camera
    // may detect other models in the scene
    // also skip duplicates (keep first detected one)
    if (!allowed(name))
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Ignoring model: %s", name.c_str());
      continue;
    }

    // retrieve the model from the world
    // (to get access to other properties if needed)
    auto model = world_->ModelByName(name);
    if (!model)
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Model %s not found in world", name.c_str());
      continue;
    }

    // Use ray to check if there is line of sight between camera and model
    // Ray goes from camera to model
    const ignition::math::Vector3d& start = cameraPose.Pos();
    const ignition::math::Vector3d& end = model->WorldPose().Pos();
    ray_->SetPoints(start, end);

    double dist_hit;
    std::string entity_name;
    ray_->GetIntersection(dist_hit, entity_name);

    // If the ray hits something before the model, then there is no line of sight
    if (!entity_name.empty() && entity_name.find(name) == std::string::npos)
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Ray from camera to %s hit %s instead", name.c_str(), entity_name.c_str());
      continue;
    }
    RCLCPP_DEBUG(ros_node_->get_logger(), "Detected person: %s", name.c_str());

    // Add to detected models set
    detected_models.insert(name);

    people_msgs::msg::Person person;
    // use the same name as in the parameter server
    person.name = getModelName(name);

    // Pose (relative to camera)
    auto& p = person.position;
    // retrieve X,Y from model world pose
    p.x = model->WorldPose().Pos().X();
    p.y = model->WorldPose().Pos().Y();

    // Get yaw from quaternion
    p.z = model->WorldPose().Rot().Yaw() - M_PI_2;
    // normalize to [-pi, pi]
    p.z = std::atan2(std::sin(p.z), std::cos(p.z));

    auto& v = person.velocity;

    if (lastUpdate != gazebo::common::Time(0))
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "lastUpdate: %.3f, current: %.3f", lastUpdate.Double(),
                   parent_sensor_->LastUpdateTime().Double());
      v = ComputeVelocity(name, model->WorldPose().Pos(), parent_sensor_->LastUpdateTime());
    }
    output_msg.people.emplace_back(std::move(person));

    // update position history only if enough time has passed
    gazebo::common::Time current_time = parent_sensor_->LastUpdateTime();
    auto last_update_it = last_position_update_time_.find(name);
    bool should_update = false;

    if (last_update_it == last_position_update_time_.end())
    {
      // First time seeing this model
      should_update = true;
    }
    else
    {
      // Check if enough time has passed since last update
      double time_diff = (current_time - last_update_it->second).Double();
      should_update = (time_diff >= MIN_UPDATE_INTERVAL);
    }

    if (should_update)
    {
      auto& history = position_history_[name];
      history.push_back({ model->WorldPose().Pos(), current_time });

      // Keep only the last MAX_HISTORY_SIZE positions
      if (history.size() > MAX_HISTORY_SIZE)
      {
        history.pop_front();
      }

      // Update the last position update time
      last_position_update_time_[name] = current_time;

      RCLCPP_DEBUG(ros_node_->get_logger(), "Updated position history for %s (history size: %zu)", name.c_str(),
                   history.size());
    }
    else
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Skipping position update for %s (insufficient time delta)", name.c_str());
    }
  }

  // Clear position history and velocity estimates of models that were not detected this time
  for (auto it = position_history_.begin(); it != position_history_.end();)
  {
    if (detected_models.find(it->first) == detected_models.end())
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Clearing position history of not detected model %s", it->first.c_str());
      velocity_estimates_.erase(it->first);         // Also clear velocity estimate
      last_position_update_time_.erase(it->first);  // Also clear last update time
      it = position_history_.erase(it);
    }
    else
    {
      ++it;
    }
  }

  if (!output_msg.people.empty())
    pub_people_->publish(output_msg);
  lastUpdate = parent_sensor_->LastUpdateTime();
}

// Velocity estimation using exponential moving average of velocities computed
// from positions that are n steps apart. This provides stable velocity estimates
// while being computationally efficient.

geometry_msgs::msg::Point PeopleDetectorPrivate::ComputeVelocity(const std::string& name,
                                                                 const ignition::math::Vector3d& current_position,
                                                                 gazebo::common::Time current_time)
{
  geometry_msgs::msg::Point velocity;

  auto it = position_history_.find(name);
  if (it == position_history_.end() || it->second.empty())
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "No position history found for %s", name.c_str());
    return velocity;  // Return zero velocity
  }

  const auto& history = it->second;

  // Need at least VELOCITY_STEP_SIZE + 1 points to compute velocity n steps apart
  if (history.size() < VELOCITY_STEP_SIZE + 1)
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Insufficient position history for %s (size: %zu, need: %zu)", name.c_str(),
                 history.size(), VELOCITY_STEP_SIZE + 1);
    return velocity;  // Return zero velocity
  }

  // Get positions n steps apart
  const auto& current_point = history[history.size() - 1];                         // Most recent position
  const auto& reference_point = history[history.size() - 1 - VELOCITY_STEP_SIZE];  // Position n steps ago

  // Compute time difference
  double dt = (current_point.timestamp - reference_point.timestamp).Double();

  if (dt <= 1e-10)
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Insufficient time difference for %s: dt=%.6f", name.c_str(), dt);
    return velocity;  // Return zero velocity
  }

  // Compute instantaneous velocity using positions n steps apart
  geometry_msgs::msg::Point current_velocity;
  current_velocity.x = (current_point.position.X() - reference_point.position.X()) / dt;
  current_velocity.y = (current_point.position.Y() - reference_point.position.Y()) / dt;

  // For angular velocity, handle angle wrapping
  double current_yaw = std::atan2(std::sin(current_point.position.Z()), std::cos(current_point.position.Z()));
  double reference_yaw = std::atan2(std::sin(reference_point.position.Z()), std::cos(reference_point.position.Z()));
  double dyaw = current_yaw - reference_yaw;
  // normalize to [-pi, pi]
  dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
  current_velocity.z = dyaw / dt;

  // Apply exponential moving average to smooth the velocity estimate
  auto vel_it = velocity_estimates_.find(name);
  if (vel_it == velocity_estimates_.end())
  {
    // First velocity estimate for this model
    velocity_estimates_[name] = current_velocity;
    velocity = current_velocity;
    RCLCPP_DEBUG(ros_node_->get_logger(), "Initial velocity for %s: vx=%.3f, vy=%.3f, wz=%.3f (dt=%.3f)", name.c_str(),
                 velocity.x, velocity.y, velocity.z, dt);
  }
  else
  {
    // Apply exponential moving average: new_estimate = alpha * current + (1-alpha) * previous
    auto& prev_velocity = vel_it->second;
    velocity.x = EMA_ALPHA * current_velocity.x + (1.0 - EMA_ALPHA) * prev_velocity.x;
    velocity.y = EMA_ALPHA * current_velocity.y + (1.0 - EMA_ALPHA) * prev_velocity.y;
    velocity.z = EMA_ALPHA * current_velocity.z + (1.0 - EMA_ALPHA) * prev_velocity.z;

    // Update stored estimate
    velocity_estimates_[name] = velocity;

    RCLCPP_DEBUG(ros_node_->get_logger(),
                 "Smoothed velocity for %s: vx=%.3f, vy=%.3f, wz=%.3f (current: %.3f,%.3f,%.3f, dt=%.3f)", name.c_str(),
                 velocity.x, velocity.y, velocity.z, current_velocity.x, current_velocity.y, current_velocity.z, dt);
  }

  return velocity;
}

void PeopleDetectorPrivate::GetAgentsList()
{
  auto ctx = ros_node_->get_node_base_interface()->get_context();
  auto param_node = std::make_shared<rclcpp::Node>("people_detector_param_client", rclcpp::NodeOptions().context(ctx));

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(param_node, "hunav_loader");
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(ros_node_->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(ros_node_->get_logger(), "Reading parameters...");
  auto parameters = parameters_client->get_parameters({ "agents" });
  if (parameters.empty())
  {
    RCLCPP_WARN(ros_node_->get_logger(), "No agents parameter found on parameter server");
    return;
  }
  agents_names_ = parameters[0].as_string_array();
  RCLCPP_INFO(ros_node_->get_logger(), "Got %zu agents", agents_names_.size());
  if (agents_names_.empty())
  {
    RCLCPP_WARN(ros_node_->get_logger(), "No agents found on parameter server");
    return;
  }
  for (auto& name : agents_names_)
    RCLCPP_INFO(ros_node_->get_logger(), "Will detect agent: %s", name.c_str());
  param_node.reset();
}

bool PeopleDetectorPrivate::allowed(const std::string& name) const
{
  if (agents_names_.empty())
    return true;
  return std::any_of(agents_names_.begin(), agents_names_.end(),
                     [&name](const std::string& agent_name) { return name.find(agent_name) != std::string::npos; });
}

std::string PeopleDetectorPrivate::getModelName(const std::string& name) const
{
  if (agents_names_.empty())
    return name;
  for (auto& agent_name : agents_names_)
  {
    // the name may contain a suffix, e.g., "agent0_body"
    if (name.find(agent_name) != std::string::npos)
      return agent_name;
  }
  return name;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(PeopleDetector)
}  // namespace gazebo_people_detector
