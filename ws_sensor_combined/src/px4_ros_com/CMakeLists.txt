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

# demos/sensor_accel
add_executable(sensor_accel src/demos/sensor_accel.cpp)
ament_target_dependencies(sensor_accel rclcpp px4_msgs)
install(TARGETS sensor_accel DESTINATION lib/${PROJECT_NAME})

# demos/sensor_baro
add_executable(sensor_baro src/demos/sensor_baro.cpp)
ament_target_dependencies(sensor_baro rclcpp px4_msgs)
install(TARGETS sensor_baro DESTINATION lib/${PROJECT_NAME})

# demos/sensor_combined
add_executable(sensor_combined src/demos/sensor_combined.cpp)
ament_target_dependencies(sensor_combined rclcpp px4_msgs)
install(TARGETS sensor_combined DESTINATION lib/${PROJECT_NAME})

# demos/sensor_gps
add_executable(sensor_gps src/demos/sensor_gps.cpp)
ament_target_dependencies(sensor_gps rclcpp px4_msgs)
install(TARGETS sensor_gps DESTINATION lib/${PROJECT_NAME})

# demos/sensor_gyro
add_executable(sensor_gyro src/demos/sensor_gyro.cpp)
ament_target_dependencies(sensor_gyro rclcpp px4_msgs)
install(TARGETS sensor_gyro DESTINATION lib/${PROJECT_NAME})

# demos/sensor_mag
add_executable(sensor_mag src/demos/sensor_mag.cpp)
ament_target_dependencies(sensor_mag rclcpp px4_msgs)
install(TARGETS sensor_mag DESTINATION lib/${PROJECT_NAME})

# demos/vehicle_angular_velocity
add_executable(vehicle_angular_velocity src/demos/vehicle_angular_velocity.cpp)
ament_target_dependencies(vehicle_angular_velocity rclcpp px4_msgs)
install(TARGETS vehicle_angular_velocity DESTINATION lib/${PROJECT_NAME})

# demos/vehicle_attitude
add_executable(vehicle_attitude src/demos/vehicle_attitude.cpp)
ament_target_dependencies(vehicle_attitude rclcpp px4_msgs)
install(TARGETS vehicle_attitude DESTINATION lib/${PROJECT_NAME})

# demos/vehicle_global_position
add_executable(vehicle_global_position src/demos/vehicle_global_position.cpp)
ament_target_dependencies(vehicle_global_position rclcpp px4_msgs)
install(TARGETS vehicle_global_position DESTINATION lib/${PROJECT_NAME})

# demos/vehicle_local_position
add_executable(vehicle_local_position src/demos/vehicle_local_position.cpp)
ament_target_dependencies(vehicle_local_position rclcpp px4_msgs)
install(TARGETS vehicle_local_position DESTINATION lib/${PROJECT_NAME})

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
