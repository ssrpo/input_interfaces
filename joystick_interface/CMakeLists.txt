cmake_minimum_required(VERSION 3.8)
project(joystick_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


### Franka dependencies
set(FRANKA_DEPS
  franka_msgs)
### Franka specific dependencies
foreach(dep IN ITEMS ${FRANKA_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopCmd.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include header directories
include_directories(
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Add the executable for the teleoperation node
add_executable(joystick_input_node
  src/main.cpp
  src/joystick_input.cpp
)

# Package dependencies
ament_target_dependencies(joystick_input_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Link generated message code
rosidl_target_interfaces(joystick_input_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Ensure executables depend on the message target
add_dependencies(joystick_input_node ${PROJECT_NAME}__rosidl_typesupport_cpp)


# Add the executable for the franka gripper
add_executable(franka_gripper_node
  src/franka_gripper.cpp
)

# Package dependencies
ament_target_dependencies(franka_gripper_node
  rclcpp
  rclcpp_action
  ${FRANKA_DEPS}
)

# Link generated message code
rosidl_target_interfaces(franka_gripper_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_export_dependencies(rosidl_default_runtime)

# Ensure executables depend on the message target
add_dependencies(franka_gripper_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install the executable
install(TARGETS
  joystick_input_node
  franka_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, and rviz directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()