#ifndef GAZEBO_PEOPLE_DETECTOR_HPP_
#define GAZEBO_PEOPLE_DETECTOR_HPP_

// C++
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <regex>
#include <optional>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

// ROS
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "people_msgs/msg/people.hpp"

namespace gazebo_people_detector
{

class PeopleDetectorPrivate;

class PeopleDetector : public gazebo::SensorPlugin

{
public:
  /// \brief Constructor
  PeopleDetector();

  /// \brief Destructor
  virtual ~PeopleDetector();

  /// \brief Load the sensor
  virtual void Load(const gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<PeopleDetectorPrivate> impl_;
};

}  // namespace gazebo_people_detector

#endif
