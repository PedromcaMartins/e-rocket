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

    constexpr char CONTROLLER_YAW_ACTIVE_PARAM[] = "offboard.controller.yaw.active";
    constexpr char CONTROLLER_YAW_K_P_PARAM[] = "offboard.controller.yaw.gains.k_p";
    constexpr char CONTROLLER_YAW_K_D_PARAM[] = "offboard.controller.yaw.gains.k_d";
    constexpr char CONTROLLER_YAW_K_I_PARAM[] = "offboard.controller.yaw.gains.k_i";

    constexpr char CONTROLLER_FREQUENCY_HERTZ_PARAM[] = "offboard.controller.frequency_hertz";
    constexpr char CONTROLLER_MOTOR_THRUST_PERCENTAGE_PARAM[] = "offboard.controller.motor_thrust_percentage";

    constexpr char CONTROLLER_THRUST_CURVE_M_PARAM[] = "offboard.controller.motor_thrust_curve.m";
    constexpr char CONTROLLER_THRUST_CURVE_B_PARAM[] = "offboard.controller.motor_thrust_curve.b";

} // namespace controller

namespace simulator
{

    constexpr char MASS_OF_SYSTEM[]             = "offboard.simulator.mass_of_system";
    constexpr char LENGTH_OF_PENDULUM[]         = "offboard.simulator.length_of_pendulum";
    constexpr char GRAVITATIONAL_ACCELERATION[] = "offboard.simulator.gravitational_acceleration";
    constexpr char MOMENT_OF_INERTIA[]          = "offboard.simulator.moment_of_inertia";
    
} // namespace simulator

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

    constexpr char MOCAP_TOPIC[]         = "/mocap/pose_enu/e_rocket";
    constexpr char MOCAP_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.output.mocap";

    constexpr char SERVOS_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.output.servos";
    constexpr char MOTORS_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.output.motors";
    constexpr char ATTITUDE_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.input.attitude";
    constexpr char ANGULAR_RATE_MAPPING_PARAM[] = "offboard.px4_ros2_message_mapping.input.angular_rate";

} // namespace px4_ros2_message_mapping

namespace mission
{

    constexpr char MISSION_SETPOINT_PARAM[] = "offboard.mission.setpoint";

} // namespace mission

} // namespace constants
} // namespace one_degree_freedom
