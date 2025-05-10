cmake_minimum_required(VERSION 3.5)
project(px4_ros_demos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

#################
# Setup targets #
#################

# actuator_motors
add_executable(actuator_motors src/actuator_motors.cpp)
ament_target_dependencies(actuator_motors rclcpp px4_msgs)
install(TARGETS actuator_motors DESTINATION lib/${PROJECT_NAME})

# actuator_servos
add_executable(actuator_servos src/actuator_servos.cpp)
ament_target_dependencies(actuator_servos rclcpp px4_msgs)
install(TARGETS actuator_servos DESTINATION lib/${PROJECT_NAME})

# offboard_control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# sensor_combined
add_executable(sensor_combined src/sensor_combined.cpp)
ament_target_dependencies(sensor_combined rclcpp px4_msgs)
install(TARGETS sensor_combined DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
