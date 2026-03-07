cmake_minimum_required(VERSION 3.8)
project(gazebo_door)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(proximity_door_plugin SHARED src/proximity_door_plugin.cpp)
target_link_libraries(proximity_door_plugin ${GAZEBO_LIBRARIES})
target_include_directories(proximity_door_plugin PUBLIC include)

ament_target_dependencies(proximity_door_plugin
  "gazebo_ros"
)

ament_export_include_directories(include)
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
  proximity_door_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_door.dsv.in")

ament_package()