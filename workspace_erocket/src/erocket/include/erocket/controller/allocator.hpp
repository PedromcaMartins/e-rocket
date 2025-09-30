#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <erocket/msg/allocator_debug.hpp>
#include <erocket/frame_transforms.h>
#include <erocket/vehicle_constants.hpp>
#include <erocket/constants.hpp>

using namespace px4_msgs::msg;
using namespace erocket::msg;
using namespace erocket::constants::controller;

namespace frame_transforms = erocket::frame_transforms;

struct ServoAllocatorInput {
    double inner_servo_tilt_angle_radians; 
    double outer_servo_tilt_angle_radians;
};

struct MotorAllocatorInput {
    double delta_motor_pwm;
    double average_motor_thrust_newtons;
};

struct ServoAllocatorOutput {
    double inner_servo_pwm;
    double outer_servo_pwm;
};

struct MotorAllocatorOutput {
    double upwards_motor_pwm;
    double downwards_motor_pwm;
};

class Allocator {
public:
    Allocator(
        rclcpp::Node* node,
        rclcpp::QoS qos,
        std::shared_ptr<VehicleConstants> vehicle_constants
    ) : vehicle_constants_(vehicle_constants),
        servo_tilt_angle_publisher_{node->create_publisher<ActuatorServos>(
            CONTROLLER_OUTPUT_SERVO_PWM_TOPIC, qos
        )}, 
        motor_thrust_publisher_{node->create_publisher<ActuatorMotors>(
            CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC, qos
        )},
        debug_publisher_{node->create_publisher<AllocatorDebug>(
            ALLOCATOR_DEBUG_TOPIC, qos
        )},
        clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    {
        compute_motor_allocation({
            NAN,
            NAN
        });

        compute_servo_allocation({
            0.0f,
            0.0f
        });
    }

    void compute_servo_allocation(ServoAllocatorInput input) {
        servo_input_ = input;

        double inner_servo_pwm = servo_curve_tilt_radians_to_pwm(input.inner_servo_tilt_angle_radians);
        double outer_servo_pwm = servo_curve_tilt_radians_to_pwm(input.outer_servo_tilt_angle_radians);

        servo_output_.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        servo_output_.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);

        publish_servo_pwm();
        publish_allocator_debug();
    }

    void compute_motor_allocation(MotorAllocatorInput input) {
        motor_input_ = input;

        double average_motor_pwm = motor_thrust_curve_newtons_to_pwm(input.average_motor_thrust_newtons);
        double upwards_motor_pwm = average_motor_pwm - input.delta_motor_pwm / 2.0f;
        double downwards_motor_pwm = average_motor_pwm + input.delta_motor_pwm / 2.0f;

        motor_output_.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        motor_output_.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

        publish_motor_pwm();
        publish_allocator_debug();
    }

    double motor_thrust_curve_pwm_to_newtons(double motor_pwm) {
        auto g = vehicle_constants_->gravitational_acceleration_;
        auto thrust_curve_m = vehicle_constants_->motor_thrust_curve_m_;
        auto thrust_curve_b = vehicle_constants_->motor_thrust_curve_b_;

        return (motor_pwm * thrust_curve_m + thrust_curve_b) / 1000.0f * g;
    }

    double motor_thrust_curve_newtons_to_pwm(double thrust_newtons) {
        auto g = vehicle_constants_->gravitational_acceleration_;
        auto thrust_curve_m = vehicle_constants_->motor_thrust_curve_m_;
        auto thrust_curve_b = vehicle_constants_->motor_thrust_curve_b_;

        return ((thrust_newtons * 1000.0f) / g - thrust_curve_b) / thrust_curve_m;
    }

private: 
    std::shared_ptr<VehicleConstants> vehicle_constants_;

    ServoAllocatorInput servo_input_;
    MotorAllocatorInput motor_input_;
    ServoAllocatorOutput servo_output_;
    MotorAllocatorOutput motor_output_;

	rclcpp::Publisher<ActuatorServos>::SharedPtr    servo_tilt_angle_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr    motor_thrust_publisher_;
    rclcpp::Publisher<AllocatorDebug>::SharedPtr    debug_publisher_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_ = rclcpp::get_logger("allocator");

    double limit_range_servo_pwm(double servo_pwm) {
        servo_pwm = (servo_pwm > 1.0f)? 1.0f : servo_pwm;
        servo_pwm = (servo_pwm < -1.0f)? -1.0f : servo_pwm;

        return servo_pwm;
    }

    double limit_range_motor_pwm(double motor_pwm) {
        motor_pwm = (motor_pwm > vehicle_constants_->max_motor_pwm_) ? vehicle_constants_->max_motor_pwm_ : motor_pwm;
        motor_pwm = (motor_pwm < 0.0f) ? 0.0f : motor_pwm;

        return motor_pwm;
    }

    double servo_curve_tilt_radians_to_pwm(double servo_tilt_angle_radians) {
        double servo_tilt_angle_degrees = frame_transforms::radians_to_degrees(servo_tilt_angle_radians);
        return servo_tilt_angle_degrees / vehicle_constants_->servo_max_tilt_angle_degrees_;
    }

    void publish_servo_pwm()
    {
        ActuatorServos msg{};
        msg.timestamp = clock_->now().nanoseconds() / 1000;
        if (vehicle_constants_->servo_active_) {
            msg.control[0] = servo_output_.outer_servo_pwm;
            msg.control[1] = servo_output_.inner_servo_pwm;
        } else {
            // If servos are not active, disable them
            msg.control[0] = 0.;
            msg.control[1] = 0.;
        }
        servo_tilt_angle_publisher_->publish(msg);
    }

    /**
     * @brief Publish the computed actuator motor PWM values.
     */
    void publish_motor_pwm()
    {
        ActuatorMotors msg{};
        msg.timestamp = clock_->now().nanoseconds() / 1000;
        if (vehicle_constants_->motor_active_) {
            msg.control[0] = motor_output_.upwards_motor_pwm;
            msg.control[1] = motor_output_.downwards_motor_pwm;
        } else {
            // If motors are not active, disable them
            msg.control[0] = NAN;
            msg.control[1] = NAN;
        }
        motor_thrust_publisher_->publish(msg);
    }

    void publish_allocator_debug() {
        AllocatorDebug msg{};
        msg.stamp = clock_->now();

        msg.servo_inner_tilt_angle_degrees = frame_transforms::radians_to_degrees(servo_input_.inner_servo_tilt_angle_radians);
        msg.servo_outer_tilt_angle_degrees = frame_transforms::radians_to_degrees(servo_input_.outer_servo_tilt_angle_radians);
        msg.servo_inner_pwm = servo_output_.inner_servo_pwm;
        msg.servo_outer_pwm = servo_output_.outer_servo_pwm;

        msg.motor_delta_pwm = motor_input_.delta_motor_pwm;
        msg.motor_average_pwm = motor_thrust_curve_newtons_to_pwm(motor_input_.average_motor_thrust_newtons);
        msg.motor_upwards_pwm = motor_output_.upwards_motor_pwm;
        msg.motor_downwards_pwm = motor_output_.downwards_motor_pwm;

        debug_publisher_->publish(msg);
    }
};
