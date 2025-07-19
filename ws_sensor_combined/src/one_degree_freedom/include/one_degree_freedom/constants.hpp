// constants.hpp
#pragma once

namespace one_degree_freedom
{
namespace constants
{

    constexpr char MASS_OF_SYSTEM[]             = "offboard.mass_of_system";
    constexpr char LENGTH_OF_PENDULUM[]         = "offboard.length_of_pendulum";
    constexpr char GRAVITATIONAL_ACCELERATION[] = "offboard.gravitational_acceleration";
    constexpr char MOMENT_OF_INERTIA[]          = "offboard.moment_of_inertia";

namespace controller
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char CONTROLLER_INPUT_ATTITUDE_TOPIC[] =      "/fmu/out/vehicle_attitude";
    constexpr char CONTROLLER_INPUT_ANGULAR_RATE_TOPIC[] =  "/fmu/out/vehicle_angular_velocity";
    constexpr char CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC[] =    "/fmu/in/actuator_motors";
    constexpr char CONTROLLER_OUTPUT_SERVO_PWM_TOPIC[] =    "/fmu/in/actuator_servos";

    constexpr char CONTROLLER_DEBUG_TOPIC[] = "/offboard/controller/debug";
    constexpr char ALLOCATOR_DEBUG_TOPIC[] = "/offboard/allocator/debug";

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
    constexpr char CONTROLLER_DEFAULT_MOTOR_PWM[]     = "offboard.controller.default_motor_pwm";

    constexpr char CONTROLLER_THRUST_CURVE_M_PARAM[] = "offboard.controller.motor_thrust_curve.m";
    constexpr char CONTROLLER_THRUST_CURVE_B_PARAM[] = "offboard.controller.motor_thrust_curve.b";
    constexpr char CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM[] = "offboard.controller.servo_max_tilt_angle_degrees";
    constexpr char CONTROLLER_MOTOR_MAX_PWM_PARAM[]  = "offboard.controller.motor_max_pwm";

} // namespace controller

namespace flight_mode
{

    //<! Topics for the 1-degree-of-freedom system
    constexpr char FLIGHT_MODE_GET_TOPIC[] = "offboard/flight_mode/get";
    constexpr char FLIGHT_MODE_SET_TOPIC[] = "offboard/flight_mode/set";

} // namespace flight_mode

namespace flight_mode
{

    constexpr float MANTAIN_OFFBOARD_MODE_TIMER_PERIOD_SECONDS = 0.1f; // 10 Hz

    //<! Parameters for the flight mode node
    constexpr char FLIGHT_MODE_PARAM[] = "offboard.flight_mode";

} // namespace flight_mode

namespace mocap_forwarder
{

    constexpr char MOCAP_TOPIC[]         = "/mocap/pose_enu/erocket";
    constexpr char MOCAP_ACTIVE_PARAM[]  = "offboard.mocap.active";

} // namespace mocap_forwarder

namespace setpoint
{

    constexpr char CONTROLLER_INPUT_ATTITUDE_SETPOINT_TOPIC[] = "offboard/attitude_setpoint_degrees";
    constexpr char MISSION_ATTITUDE_SETPOINT_PARAM[] = "offboard.mission.attitude_setpoint.degrees";

    constexpr char CONTROLLER_INPUT_POSITION_SETPOINT_TOPIC[] = "offboard/position_setpoint_meters";
    constexpr char MISSION_POSITION_SETPOINT_PARAM[] = "offboard.mission.position_setpoint.meters";

    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_PERIOD_PARAM[]   = "offboard.mission.sine_wave_trajectory.period";

    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_AMPLITUDE_ATTITUDE_DEGREES_PARAM[]   = "offboard.mission.sine_wave_trajectory.amplitude_attitude_degrees";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_ATTITUDE_ROLL_PARAM[]  = "offboard.mission.sine_wave_trajectory.activate_attitude_roll";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_ATTITUDE_PITCH_PARAM[] = "offboard.mission.sine_wave_trajectory.activate_attitude_pitch";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_ATTITUDE_YAW_PARAM[]   = "offboard.mission.sine_wave_trajectory.activate_attitude_yaw";

    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_AMPLITUDE_POSITION_METERS_PARAM[]   = "offboard.mission.sine_wave_trajectory.amplitude_position_meters";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_POSITION_X_PARAM[] = "offboard.mission.sine_wave_trajectory.activate_position_x";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_POSITION_Y_PARAM[] = "offboard.mission.sine_wave_trajectory.activate_position_y";
    constexpr char MISSION_SETPOINT_SINE_WAVE_TRAJECTORY_ACTIVE_POSITION_Z_PARAM[] = "offboard.mission.sine_wave_trajectory.activate_position_z";

} // namespace setpoint

} // namespace constants
} // namespace one_degree_freedom
