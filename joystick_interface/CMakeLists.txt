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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(extender_msgs REQUIRED)

### Franka dependencies
set(FRANKA_DEPS
  franka_msgs)
### Franka specific dependencies
foreach(dep IN ITEMS ${FRANKA_DEPS})
  find_package(${dep} REQUIRED)
endforeach()


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
  extender_msgs
)

target_include_directories(joystick_input_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

target_include_directories(franka_gripper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Also, ensure you install the include directory if you plan to use these headers elsewhere
install(DIRECTORY include/
  DESTINATION include
)

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