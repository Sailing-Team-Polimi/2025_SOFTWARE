cmake_minimum_required(VERSION 3.8)
project(flap_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sail_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

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

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Create the executables
add_executable(wand2servo src/wand2servo.cpp)
ament_target_dependencies(wand2servo rclcpp std_msgs sail_msgs)

# Create the executables
add_executable(wand_direct src/wand_direct.cpp)
ament_target_dependencies(wand_direct rclcpp std_msgs sail_msgs)

install(TARGETS
  wand2servo
  wand_direct
  DESTINATION lib/${PROJECT_NAME}
)

# Create the pot2wand executable
# add_executable(pot2wand src/pot2wand.cpp)
# ament_target_dependencies(pot2wand rclcpp std_msgs)
# 
# install(TARGETS
#   pot2wand
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
