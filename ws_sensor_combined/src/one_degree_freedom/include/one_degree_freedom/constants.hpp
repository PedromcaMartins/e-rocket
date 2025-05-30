// constants.hpp
#pragma once

namespace one_degree_freedom
{
namespace constants
{
namespace controller
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char CONTROLLER_INPUT_ATTITUDE_TOPIC[] = "offboard/controller_input_attitude";
    constexpr char CONTROLLER_INPUT_ANGULAR_RATE_TOPIC[] = "offboard/controller_input_angular_rate";
    constexpr char CONTROLLER_INPUT_SETPOINT_TOPIC[] = "offboard/controller_input_setpoint";
    constexpr char CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC[] = "offboard/controller_output_servo_tilt_angle";
    constexpr char CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC[] = "offboard/controller_output_motor_thrust";

    //<! Parameters for the 1-degree-of-freedom controller
    constexpr char CONTROLLER_INPUT_SETPOINT_PARAM[] = "controller_input_setpoint_radians";

    //<! Constants for the 1-degree-of-freedom system
    constexpr float M = 2.0f; // mass of the system
    constexpr float L = 0.5f; // length of the pendulum
    constexpr float G = 9.81f; // gravitational acceleration
    constexpr float J = 0.3750f; // moment of inertia

    constexpr float CONTROLLER_DT_SECONDS = 0.02f; // 50 Hz
    constexpr float CONTROLLER_MOTOR_THRUST_PERCENTAGE = 0.1f;

} // namespace controller

namespace px4_ros2_flight_mode
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char FLIGHT_MODE_REQUEST_TOPIC[] = "offboard/flight_mode/request";
    constexpr char FLIGHT_MODE_RESPONSE_TOPIC[] = "offboard/flight_mode/response";

    //<! Flight modes for the PX4
    constexpr char FLIGHT_MODE_OFFBOARD[] = "OFFBOARD";
    constexpr char FLIGHT_MODE_MANUAL[] = "MANUAL";
    constexpr char FLIGHT_MODE_ARM[] = "ARM";
    constexpr char FLIGHT_MODE_DISARM[] = "DISARM";

    constexpr float MANTAIN_OFFBOARD_MODE_TIMER_PERIOD_SECONDS = 0.1f; // 10 Hz

} // namespace px4_ros2_flight_mode
} // namespace constants
} // namespace one_degree_freedom
