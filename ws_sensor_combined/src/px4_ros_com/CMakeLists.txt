cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

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

# ##############################################################################
# Generate ROS messages, ROS2 interfaces and IDL files #
# ##############################################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB PX4_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
	${PX4_MSGS}
	${PX4_SRVS}
	DEPENDENCIES builtin_interfaces
)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 1_degree_freedom/controller_tester
add_executable(1_degree_freedom_controller_tester src/1_degree_freedom/controller_tester.cpp)
ament_target_dependencies(1_degree_freedom_controller_tester rclcpp px4_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(1_degree_freedom_controller_tester ${cpp_typesupport_target})
install(TARGETS 1_degree_freedom_controller_tester DESTINATION lib/${PROJECT_NAME})

# 1_degree_freedom/controller
add_executable(1_degree_freedom_controller src/1_degree_freedom/controller.cpp)
ament_target_dependencies(1_degree_freedom_controller rclcpp px4_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(1_degree_freedom_controller ${cpp_typesupport_target})
target_include_directories(1_degree_freedom_controller PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS 1_degree_freedom_controller DESTINATION lib/${PROJECT_NAME})

# demos/actuator_motors
add_executable(actuator_motors src/demos/actuator_motors.cpp)
ament_target_dependencies(actuator_motors rclcpp px4_msgs)
install(TARGETS actuator_motors DESTINATION lib/${PROJECT_NAME})

# demos/actuator_servos
add_executable(actuator_servos src/demos/actuator_servos.cpp)
ament_target_dependencies(actuator_servos rclcpp px4_msgs)
install(TARGETS actuator_servos DESTINATION lib/${PROJECT_NAME})

# demos/offboard_control
add_executable(offboard_control src/demos/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# demos/sensor_combined
add_executable(sensor_combined src/demos/sensor_combined.cpp)
ament_target_dependencies(sensor_combined rclcpp px4_msgs)
install(TARGETS sensor_combined DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/offboard_py/offboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
