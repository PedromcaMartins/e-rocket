#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/constants.hpp>

using namespace one_degree_freedom::constants;
using namespace one_degree_freedom::constants::vehicle;

class VehicleConstants {
public:
    double mass_of_system_;
    double length_of_pendulum_;
    double gravitational_acceleration_;
    double moment_of_inertia_;

    bool servo_active_;
    bool motor_active_;
    double servo_max_tilt_angle_degrees_;
    double default_motor_pwm_;
    double max_motor_pwm_;
    double motor_thrust_curve_m_;
    double motor_thrust_curve_b_;

    VehicleConstants(rclcpp::Node* node) {
        node->declare_parameter<double>(MASS_OF_SYSTEM);
        mass_of_system_ = node->get_parameter(MASS_OF_SYSTEM).as_double();
        if (mass_of_system_ <= 0.0f || std::isnan(mass_of_system_)) {
            RCLCPP_ERROR(this->logger_, "Could not read mass of system correctly.");
            throw std::runtime_error("Mass of system invalid");
        }
        RCLCPP_INFO(this->logger_, "Mass of system: %f", mass_of_system_);

        node->declare_parameter<double>(LENGTH_OF_PENDULUM);
        length_of_pendulum_ = node->get_parameter(LENGTH_OF_PENDULUM).as_double();
        if (length_of_pendulum_ <= 0.0f || std::isnan(length_of_pendulum_)) {
            RCLCPP_ERROR(this->logger_, "Could not read length of pendulum correctly.");
            throw std::runtime_error("Length of pendulum invalid");
        }
        RCLCPP_INFO(this->logger_, "Length of pendulum: %f", length_of_pendulum_);

        node->declare_parameter<double>(GRAVITATIONAL_ACCELERATION);
        gravitational_acceleration_ = node->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        if (gravitational_acceleration_ <= 0.0f || std::isnan(gravitational_acceleration_)) {
            RCLCPP_ERROR(this->logger_, "Could not read gravitational acceleration correctly.");
            throw std::runtime_error("Gravitational acceleration invalid");
        }
        RCLCPP_INFO(this->logger_, "Gravitational acceleration: %f", gravitational_acceleration_);

        node->declare_parameter<double>(MOMENT_OF_INERTIA);
        moment_of_inertia_ = node->get_parameter(MOMENT_OF_INERTIA).as_double();
        if (moment_of_inertia_ <= 0.0f || std::isnan(moment_of_inertia_)) {
            RCLCPP_ERROR(this->logger_, "Could not read moment of inertia correctly.");
            throw std::runtime_error("Moment of inertia invalid");
        }
        RCLCPP_INFO(this->logger_, "Moment of inertia: %f", moment_of_inertia_);

        node->declare_parameter<bool>(SERVO_ACTIVE_PARAM);
        node->declare_parameter<bool>(MOTOR_ACTIVE_PARAM);
        servo_active_ = node->get_parameter(SERVO_ACTIVE_PARAM).as_bool();
        motor_active_ = node->get_parameter(MOTOR_ACTIVE_PARAM).as_bool();
        RCLCPP_INFO(logger_, "Motor %s; Servo %s", 
            (motor_active_)? "active" : "off",
            (servo_active_)? "active" : "off"
        );

        node->declare_parameter<double>(SERVO_MAX_TILT_ANGLE_DEGREES_PARAM);
        servo_max_tilt_angle_degrees_ = node->get_parameter(SERVO_MAX_TILT_ANGLE_DEGREES_PARAM).as_double();
        if (servo_max_tilt_angle_degrees_ < 0.0f || servo_max_tilt_angle_degrees_ > 90.0f || std::isnan(servo_max_tilt_angle_degrees_)) {
            RCLCPP_ERROR(this->logger_, "Could not read servo max tilt angle correctly.");
            throw std::runtime_error("Servo max tilt angle invalid");
        }
        RCLCPP_INFO(this->logger_, "Servo max tilt angle (degrees): %f", servo_max_tilt_angle_degrees_);

        node->declare_parameter<double>(MAX_MOTOR_PWM_PARAM);
        max_motor_pwm_ = node->get_parameter(MAX_MOTOR_PWM_PARAM).as_double();
        if (max_motor_pwm_ <= 0.0f || max_motor_pwm_ > 1.0f || std::isnan(max_motor_pwm_)) {
            RCLCPP_ERROR(this->logger_, "Could not read max motor pwm correctly.");
            throw std::runtime_error("Max motor pwm invalid");
        }
        RCLCPP_INFO(this->logger_, "Max motor pwm: %f", max_motor_pwm_);

        node->declare_parameter<double>(CONTROLLER_DEFAULT_MOTOR_PWM);
        default_motor_pwm_ = node->get_parameter(CONTROLLER_DEFAULT_MOTOR_PWM).as_double();
        if (default_motor_pwm_ < 0.0f || default_motor_pwm_ > max_motor_pwm_ || std::isnan(default_motor_pwm_)) {
            RCLCPP_ERROR(this->logger_, "Could not read default motor pwm correctly.");
            throw std::runtime_error("Default motor pwm invalid");
        }
        RCLCPP_INFO(this->logger_, "Default motor pwm: %f", default_motor_pwm_);

        node->declare_parameter<double>(MOTOR_THRUST_CURVE_M_PARAM);
        motor_thrust_curve_m_ = node->get_parameter(MOTOR_THRUST_CURVE_M_PARAM).as_double();
        if (motor_thrust_curve_m_ <= 0.0f || std::isnan(motor_thrust_curve_m_)) {
            RCLCPP_ERROR(this->logger_, "Could not read motor thrust curve m correctly.");
            throw std::runtime_error("Motor thrust curve m invalid");
        }

        node->declare_parameter<double>(MOTOR_THRUST_CURVE_B_PARAM);
        motor_thrust_curve_b_ = node->get_parameter(MOTOR_THRUST_CURVE_B_PARAM).as_double();
        if (std::isnan(motor_thrust_curve_b_)) {
            RCLCPP_ERROR(this->logger_, "Could not read motor thrust curve b correctly.");
            throw std::runtime_error("Motor thrust curve b invalid");
        }
        RCLCPP_INFO(this->logger_, "Motor thrust curve: f(x) = %fx + %f", motor_thrust_curve_m_, motor_thrust_curve_b_);
    }

private:
    rclcpp::Logger logger_{rclcpp::get_logger("VehicleConstants")};
};
