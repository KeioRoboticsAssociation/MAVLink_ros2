cmake_minimum_required(VERSION 3.8)
project(stm32_mavlink_msgs)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Message files
set(msg_files
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "msg/EncoderState.msg"
  "msg/EncoderConfig.msg"
  "msg/RobomasterMotorCommand.msg"
  "msg/RobomasterMotorState.msg"
  "msg/RobomasterMotorConfig.msg"
  "msg/DCMotorCommand.msg"
  "msg/DCMotorState.msg"
  "msg/DCMotorConfig.msg"
)

# Service files
set(srv_files
  "srv/SetServoConfig.srv"
  "srv/SetEncoderConfig.srv"
  "srv/SetRobomasterMotorConfig.srv"
  "srv/GetRobomasterMotorConfig.srv"
  "srv/SetDCMotorConfig.srv"
  "srv/GetDCMotorConfig.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
