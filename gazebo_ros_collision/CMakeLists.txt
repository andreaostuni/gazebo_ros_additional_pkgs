cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_collision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_collision_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_ros_collision SHARED
  src/gazebo_ros_collision.cpp
)
target_include_directories(gazebo_ros_collision PUBLIC include)
target_link_libraries(gazebo_ros_collision
  ContactPlugin
)
ament_target_dependencies(gazebo_ros_collision
  "gazebo_ros"
  "gazebo_msgs"
  "gazebo_collision_msgs"
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
ament_export_libraries(gazebo_ros_collision)

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
  gazebo_ros_collision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()



