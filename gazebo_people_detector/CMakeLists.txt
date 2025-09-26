cmake_minimum_required(VERSION 3.8)
project(gazebo_people_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(people_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_people_detector SHARED
  src/gazebo_people_detector.cpp
)
target_include_directories(gazebo_people_detector PUBLIC include)
target_link_libraries(gazebo_people_detector
  ${gazebo_dev_LIBRARIES}
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(gazebo_people_detector
  "gazebo_ros"
  "gazebo_msgs"
  "people_msgs"
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
  ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  endif()
  
  install(DIRECTORY include/
  DESTINATION include)
  
  install(TARGETS
  gazebo_people_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
  ament_export_libraries(gazebo_people_detector)
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()



