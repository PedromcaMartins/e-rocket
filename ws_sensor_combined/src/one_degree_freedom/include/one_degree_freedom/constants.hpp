// constants.hpp
#pragma once

namespace one_degree_freedom
{
namespace constants
{
namespace controller
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char CONTROLLER_INPUT_ATTITUDE_TOPIC[] = "offboard/controller/input/attitude";
    constexpr char CONTROLLER_INPUT_ANGULAR_RATE_TOPIC[] = "offboard/controller/input/angular_rate";
    constexpr char CONTROLLER_INPUT_SETPOINT_TOPIC[] = "offboard/controller/input/setpoint";
    constexpr char CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC[] = "offboard/controller/output/servo_tilt_angle";
    constexpr char CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC[] = "offboard/controller/output/motor_thrust";

    constexpr char CONTROLLER_DEBUG_TOPIC[] = "/offboard/controller/debug";

    constexpr char CONTROLLER_ROLL_ACTIVE_PARAM[] = "offboard.controller.roll.active";
    constexpr char CONTROLLER_ROLL_K_P_PARAM[] = "offboard.controller.roll.gains.k_p";
    constexpr char CONTROLLER_ROLL_K_D_PARAM[] = "offboard.controller.roll.gains.k_d";
    constexpr char CONTROLLER_ROLL_K_I_PARAM[] = "offboard.controller.roll.gains.k_i";

    constexpr char CONTROLLER_PITCH_ACTIVE_PARAM[] = "offboard.controller.pitch.active";
    constexpr char CONTROLLER_PITCH_K_P_PARAM[] = "offboard.controller.pitch.gains.k_p";
    constexpr char CONTROLLER_PITCH_K_D_PARAM[] = "offboard.controller.pitch.gains.k_d";
    constexpr char CONTROLLER_PITCH_K_I_PARAM[] = "offboard.controller.pitch.gains.k_i";

    //<! Constants for the 1-degree-of-freedom system
    constexpr float M = 2.0f; // mass of the system
    constexpr float L = 0.5f; // length of the pendulum
    constexpr float G = 9.81f; // gravitational acceleration
    constexpr float J = 0.3750f; // moment of inertia

    constexpr float CONTROLLER_DT_SECONDS = 0.02f; // 50 Hz
    constexpr float CONTROLLER_MOTOR_THRUST_PERCENTAGE = 0.1f;

} // namespace controller

namespace flight_mode
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char FLIGHT_MODE_GET_TOPIC[] = "offboard/flight_mode/get";
    constexpr char FLIGHT_MODE_SET_TOPIC[] = "offboard/flight_mode/set";

} // namespace flight_mode

namespace px4_ros2_flight_mode
{

    constexpr float MANTAIN_OFFBOARD_MODE_TIMER_PERIOD_SECONDS = 0.1f; // 10 Hz

    //<! Parameters for the flight mode node
    constexpr char FLIGHT_MODE_PARAM[] = "offboard.flight_mode";

} // namespace px4_ros2_flight_mode

namespace px4_ros2_message_mapping
{

    constexpr char SERVOS_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.output.servos";
    constexpr char MOTORS_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.output.motors";
    constexpr char ATTITUDE_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.input.attitude";
    constexpr char ANGULAR_RATE_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.input.angular_rate";

} // namespace px4_ros2_message_mapping

namespace mission
{

    constexpr char MISSION_SETPOINT_PARAM[] = "offboard.mission.setpoint.radians";

} // namespace mission

} // namespace constants
} // namespace one_degree_freedom
