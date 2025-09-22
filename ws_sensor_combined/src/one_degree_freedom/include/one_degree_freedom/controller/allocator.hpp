#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/frame_transforms.h>
#include <one_degree_freedom/vehicle_constants.hpp>

namespace frame_transforms = one_degree_freedom::frame_transforms;

class Allocator {
public:
    // The pwm values are limited by their operating range
    struct ServoAllocatorOutput {
        double inner_servo_pwm;
        double outer_servo_pwm;
    };

    struct MotorAllocatorOutput {
        double upwards_motor_pwm;
        double downwards_motor_pwm;
    };

    Allocator(std::shared_ptr<VehicleConstants> vehicle_constants) : 
        thrust_curve_m_(vehicle_constants->motor_thrust_curve_m_), 
        thrust_curve_b_(vehicle_constants->motor_thrust_curve_b_), 
        g_(vehicle_constants->gravitational_acceleration_), 
        servo_max_tilt_angle_degrees_(vehicle_constants->servo_max_tilt_angle_degrees_), 
        motor_max_pwm_(vehicle_constants->max_motor_pwm_)
        { }

    ServoAllocatorOutput compute_servo_allocation(
        double inner_servo_tilt_angle_radians, 
        double outer_servo_tilt_angle_radians
    ) {
        ServoAllocatorOutput output;

        double inner_servo_pwm = servo_curve_tilt_radians_to_pwm(inner_servo_tilt_angle_radians);
        double outer_servo_pwm = servo_curve_tilt_radians_to_pwm(outer_servo_tilt_angle_radians);

        output.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        output.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);

        return output;
    }

    MotorAllocatorOutput compute_motor_allocation(
        double delta_motor_pwm, 
        double average_motor_thrust_newtons
    ) {
        MotorAllocatorOutput output;

        double average_motor_pwm = motor_thrust_curve_newtons_to_pwm(average_motor_thrust_newtons);
        double upwards_motor_pwm = average_motor_pwm - delta_motor_pwm / 2.0f;
        double downwards_motor_pwm = average_motor_pwm + delta_motor_pwm / 2.0f;

        output.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        output.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

        return output;
    }

    double motor_thrust_curve_pwm_to_newtons(double motor_pwm) {
        return (motor_pwm * thrust_curve_m_ + thrust_curve_b_) / 1000.0f * g_;
    }

    double motor_thrust_curve_newtons_to_pwm(double thrust_newtons) {
        return ((thrust_newtons * 1000.0f) / g_ - thrust_curve_b_) / thrust_curve_m_;
    }

private: 
    double thrust_curve_m_;
    double thrust_curve_b_;
    double g_;

    double servo_max_tilt_angle_degrees_;
    double motor_max_pwm_;

    double limit_range_servo_pwm(double servo_pwm) {
        servo_pwm = (servo_pwm > 1.0f)? 1.0f : servo_pwm;
        servo_pwm = (servo_pwm < -1.0f)? -1.0f : servo_pwm;

        return servo_pwm;
    }

    double limit_range_motor_pwm(double motor_pwm) {
        motor_pwm = (motor_pwm > motor_max_pwm_)? motor_max_pwm_ : motor_pwm;
        motor_pwm = (motor_pwm < 0.0f)? 0.0f : motor_pwm;

        return motor_pwm;
    }

    double servo_curve_tilt_radians_to_pwm(double servo_tilt_angle_radians) {
        double servo_tilt_angle_degrees = frame_transforms::radians_to_degrees(servo_tilt_angle_radians);
        return servo_tilt_angle_degrees / servo_max_tilt_angle_degrees_;
    }

};
