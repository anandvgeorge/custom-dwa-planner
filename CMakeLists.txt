cmake_minimum_required(VERSION 3.8)
project(custom_dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  SHARED
  src/custom_dwa_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/ 
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME} 
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}) 
ament_export_dependencies(
  rclcpp
  nav2_core
  geometry_msgs
  nav_msgs
  pluginlib
  nav2_costmap_2d
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
