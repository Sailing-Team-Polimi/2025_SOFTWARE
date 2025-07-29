cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sail_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

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

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Install include files
#install(
#  DIRECTORY include/${PROJECT_NAME}
#  DESTINATION include/
#)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)




add_executable(gps_to_local 
  src/gps_to_local.cpp
)
ament_target_dependencies(gps_to_local
   rclcpp
   sensor_msgs
   nav_msgs
   geometry_msgs
   visualization_msgs
)

add_executable(gps_altitude
  src/gps_altitude.cpp
)

ament_target_dependencies(gps_altitude
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  )


add_executable(local_to_gps
  src/local_to_gps.cpp
)

ament_target_dependencies(local_to_gps
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  )



add_executable(fix_gps_header_node 
  src/fix_gps_header_node.cpp
)
ament_target_dependencies(fix_gps_header_node 
  rclcpp 
  sensor_msgs
)

add_executable(ride_height_node 
  src/ride_height_node.cpp
)
ament_target_dependencies(ride_height_node
   rclcpp
   std_msgs
   sensor_msgs
   nav_msgs
   geometry_msgs
   visualization_msgs
   tf2_ros
   tf2_geometry_msgs
   tf2
)

add_executable(pot_to_wand_node 
  src/pot_to_wand_node.cpp
)
ament_target_dependencies(pot_to_wand_node
   rclcpp
   std_msgs
)

install(TARGETS
  fix_gps_header_node
  gps_to_local
  local_to_gps
  gps_altitude
  ride_height_node
  pot_to_wand_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
