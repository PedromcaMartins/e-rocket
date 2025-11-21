#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <erocket/msg/allocator_debug.hpp>
#include <erocket/frame_transforms.h>
#include <erocket/vehicle_constants.hpp>
#include <erocket/constants.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace erocket::msg;
using namespace erocket::constants::controller;

namespace frame_transforms = erocket::frame_transforms;

struct AllocatorInput {
    Eigen::Vector3d thrust_vector;
    double tau_delta_bar;
};

struct ServoAllocatorInput {
    double inner_servo_tilt_angle_radians; 
    double outer_servo_tilt_angle_radians;
};

struct MotorAllocatorInput {
    double delta_motor_pwm;
    double average_motor_thrust_newtons;
};

struct AllocatorOutput {
    double inner_servo_pwm;
    double outer_servo_pwm;
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
        output_ = AllocatorOutput{};

        auto t0 = node->get_clock()->now();
        while (node->get_clock()->now() - t0 < 1s) {
            publish_servo_pwm();
            publish_motor_pwm();
            rclcpp::sleep_for(100ms);
        }
    }

    void compute_allocation(AllocatorInput input) {
        thrust_ = input.thrust_vector.norm();
        M_bar_ = motor_thrust_curve_newtons_to_pwm(thrust_);
        M_delta_ = delta_torque_curve(M_bar_, input.tau_delta_bar);
        auto upwards_motor_pwm = M_bar_ + M_delta_ / 2.0f;
        auto downwards_motor_pwm = M_bar_ - M_delta_ / 2.0f;

        gamma_inner_ = std::asin(input.thrust_vector[1] / thrust_);
        gamma_outer_ = std::asin(-input.thrust_vector[0] / (sqrt(thrust_ * thrust_ - input.thrust_vector[1] * input.thrust_vector[1])));
        auto inner_servo_pwm = servo_curve_tilt_radians_to_pwm(gamma_inner_);
        auto outer_servo_pwm = servo_curve_tilt_radians_to_pwm(gamma_outer_);

        output_.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        output_.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);
        output_.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        output_.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

        publish_servo_pwm();
        publish_motor_pwm();
        publish_allocator_debug();
    }

    void compute_allocation_neutral() {
        output_.inner_servo_pwm = 0.0f;
        output_.outer_servo_pwm = 0.0f;
        output_.upwards_motor_pwm = NAN;
        output_.downwards_motor_pwm = NAN;

        publish_servo_pwm();
        publish_motor_pwm();
        publish_allocator_debug();
    }

    void compute_servo_allocation(ServoAllocatorInput input) {
        double inner_servo_pwm = servo_curve_tilt_radians_to_pwm(input.inner_servo_tilt_angle_radians);
        double outer_servo_pwm = servo_curve_tilt_radians_to_pwm(input.outer_servo_tilt_angle_radians);

        output_.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        output_.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);

        publish_servo_pwm();
        publish_allocator_debug();
    }

    void compute_motor_allocation(MotorAllocatorInput input) {
        double average_motor_pwm = motor_thrust_curve_newtons_to_pwm(input.average_motor_thrust_newtons);
        double upwards_motor_pwm = average_motor_pwm - input.delta_motor_pwm / 2.0f;
        double downwards_motor_pwm = average_motor_pwm + input.delta_motor_pwm / 2.0f;

        output_.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        output_.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

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

    AllocatorInput input_;
    AllocatorOutput output_;

    double thrust_;
    double M_bar_;
    double M_delta_;
    double gamma_inner_;
    double gamma_outer_;

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

    double delta_torque_curve(double M_bar, double tau_delta_bar) {
        auto a = vehicle_constants_->delta_torque_a_;
        auto b = vehicle_constants_->delta_torque_b_;
        auto c = vehicle_constants_->delta_torque_c_;
        return tau_delta_bar / a - (b / a) * M_bar - c / a;
    }

    void publish_servo_pwm()
    {
        ActuatorServos msg{};
        msg.timestamp = clock_->now().nanoseconds() / 1000;
        if (vehicle_constants_->servo_active_) {
            msg.control[0] = output_.outer_servo_pwm;
            msg.control[1] = output_.inner_servo_pwm;
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
            msg.control[0] = output_.upwards_motor_pwm;
            msg.control[1] = output_.downwards_motor_pwm;
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

        Eigen::Map<Eigen::Vector3d>(msg.thrust_vector.data()) = input_.thrust_vector;
        msg.tau_delta_bar = input_.tau_delta_bar;
        msg.m_bar = M_bar_;
        msg.m_delta = M_delta_;
        msg.thrust = thrust_;
        msg.gamma_inner_degrees = frame_transforms::radians_to_degrees(gamma_inner_);
        msg.gamma_outer_degrees = frame_transforms::radians_to_degrees(gamma_outer_);

        msg.motor_upwards_pwm = output_.upwards_motor_pwm;
        msg.motor_downwards_pwm = output_.downwards_motor_pwm;
        msg.outer_servo_pwm = output_.inner_servo_pwm;
        msg.inner_servo_pwm = output_.outer_servo_pwm;


        debug_publisher_->publish(msg);
    }
};
