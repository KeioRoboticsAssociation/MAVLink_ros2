cmake_minimum_required(VERSION 3.8)
project(stm32_mavlink_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-address-of-packed-member)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(stm32_mavlink_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../mavlink_SDK/mavlink
)

# Create executable
add_executable(mavlink_udp_node
  src/mavlink_udp_node.cpp
  src/servo_controller.cpp
  src/encoder_interface.cpp
  src/robomaster_controller.cpp
  src/dcmotor_controller.cpp
  src/rs485motor_controller.cpp
)

# Link dependencies
ament_target_dependencies(mavlink_udp_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  stm32_mavlink_msgs
)

# Install
install(TARGETS
  mavlink_udp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()