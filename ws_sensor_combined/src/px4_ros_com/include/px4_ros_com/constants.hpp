// constants.hpp
#pragma once

namespace px4_ros_com
{
namespace constants
{
namespace one_degree_of_freedom
{

//<! Topics for the 1-degree-of-freedom system
constexpr char CONTROLLER_INPUT_ATTITUDE[] = "offboard/controller_input_attitude";
constexpr char CONTROLLER_INPUT_ANGULAR_RATE[] = "offboard/controller_input_angular_rate";
constexpr char CONTROLLER_INPUT_SETPOINT[] = "offboard/controller_input_setpoint";
constexpr char CONTROLLER_OUTPUT_TILT_ANGLE[] = "offboard/controller_output_tilt_angle";

//<! Parameters for the 1-degree-of-freedom controller
constexpr char CONTROLLER_INPUT_SETPOINT_PARAM[] = "controller_input_setpoint";

//<! Constants for the 1-degree-of-freedom system
constexpr float M = 2.0f; // mass of the system
constexpr float L = 0.5f; // length of the pendulum
constexpr float G = 9.81f; // gravitational acceleration
constexpr float J = 0.3750f; // moment of inertia

constexpr float CONTROLLER_DT = 0.02f; // 50 Hz

} // namespace one_degree_of_freedom
} // namespace constants
} // namespace px4_ros_com
