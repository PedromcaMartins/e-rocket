#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/attitude_controller_debug.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>

#include <one_degree_freedom/controller/state.hpp>
#include <one_degree_freedom/controller/setpoint.hpp>

using one_degree_freedom::frame_transforms::radians_to_degrees;

struct AttitudePIDControllerOutput {
    double inner_servo_tilt_angle;
    double outer_servo_tilt_angle;
    double delta_motor_pwm;
};

class AttitudePIDController
{
public:
    AttitudePIDController(rclcpp::Node* node, rclcpp::QoS qos, std::shared_ptr<StateAggregator> state_aggregator, std::shared_ptr<SetpointAggregator> setpoint_aggregator): 
        state_aggregator_(state_aggregator),
        setpoint_aggregator_(setpoint_aggregator),
        attitude_controller_debug_publisher_(node->create_publisher<AttitudeControllerDebug>(CONTROLLER_ATTITUDE_DEBUG_TOPIC, qos)),
        clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    {
        node->declare_parameter<double>(CONTROLLER_ATTITUDE_FREQUENCY_HERTZ_PARAM);
        double controller_freq = node->get_parameter(CONTROLLER_ATTITUDE_FREQUENCY_HERTZ_PARAM).as_double();
        if (controller_freq <= 0.0f || std::isnan(controller_freq)) {
            RCLCPP_ERROR(logger_, "Could not read controller frequency correctly.");
            throw std::runtime_error("Controller frequency invalid");
        }
        dt_ = 1.0 / controller_freq;

        node->declare_parameter<std::vector<bool>>(CONTROLLER_ATTITUDE_ACTIVE_PARAM);
        controller_active_ = node->get_parameter(CONTROLLER_ATTITUDE_ACTIVE_PARAM).as_bool_array();
        // Safety check
        if (controller_active_.size() != 3) {
            RCLCPP_ERROR(logger_, "Controller active vector size is not 3.");
            throw std::runtime_error("Controller active vector size invalid");
        }
        RCLCPP_INFO(logger_, "Controllers: roll: %s, pitch: %s, yaw: %s",
            controller_active_[0] ? "active" : "off",
            controller_active_[1] ? "active" : "off",
            controller_active_[2] ? "active" : "off"
        );

        node->declare_parameter<std::vector<double>>(CONTROLLER_ATTITUDE_K_P_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_ATTITUDE_K_D_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_ATTITUDE_K_I_PARAM);
        k_p_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_ATTITUDE_K_P_PARAM).as_double_array().data(), 3);
        k_d_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_ATTITUDE_K_D_PARAM).as_double_array().data(), 3);
        k_i_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_ATTITUDE_K_I_PARAM).as_double_array().data(), 3);
        // Safety check
        if (k_p_.size() != 3 || k_d_.size() != 3 || k_i_.size() != 3 || 
            std::isnan(k_p_[0]) || std::isnan(k_p_[1]) || std::isnan(k_p_[2]) ||
            std::isnan(k_d_[0]) || std::isnan(k_d_[1]) || std::isnan(k_d_[2]) ||
            std::isnan(k_i_[0]) || std::isnan(k_i_[1]) || std::isnan(k_i_[2])) {
            RCLCPP_ERROR(logger_, "Invalid PID gains provided.");
            throw std::runtime_error("Gains vector invalid");
        }

        RCLCPP_INFO(logger_, "controller dt: %f", dt_);

        RCLCPP_INFO(logger_, "gains k_p: [%f, %f, %f]", k_p_[0], k_p_[1], k_p_[2]);
        RCLCPP_INFO(logger_, "gains k_d: [%f, %f, %f]", k_d_[0], k_d_[1], k_d_[2]);
        RCLCPP_INFO(logger_, "gains k_i: [%f, %f, %f]", k_i_[0], k_i_[1], k_i_[2]);
    }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    AttitudePIDControllerOutput compute() {
        auto state = state_aggregator_->get_state();
        auto setpoint = setpoint_aggregator_->get_attitude_setpoint();

        Eigen::Vector3d attitude = state.euler_angles;
        attitude[2] -= origin_yaw_;

        Eigen::Vector3d error_p = attitude - setpoint.attitude;

        // Update the integrated error
        integrated_error_ += error_p * dt_; 

        // Compute PD terms
        Eigen::Vector3d p_term = k_p_.cwiseProduct(error_p);
        Eigen::Vector3d d_term = k_d_.cwiseProduct(state.angular_rate);
        Eigen::Vector3d i_term = k_i_.cwiseProduct(integrated_error_);
        Eigen::Vector3d tilt_angle = p_term + d_term + i_term;

        // Apply controller active flags
        for (int i = 0; i < 3; ++i) {
            if (!controller_active_[i]) {
                tilt_angle[i] = 0.0;
            }
        }
        output_.inner_servo_tilt_angle = tilt_angle[0];
        output_.outer_servo_tilt_angle = tilt_angle[1];
        output_.delta_motor_pwm = tilt_angle[2];

        publish_debug();
        return output_;
    }

    bool are_all_controllers_active() {
        return controller_active_[0] && controller_active_[1] && controller_active_[2];
    }

    void publish_debug() {
        auto state = state_aggregator_->get_state();
        auto setpoint = setpoint_aggregator_->get_attitude_setpoint();

        AttitudeControllerDebug msg;
        msg.stamp = clock_->now();

        msg.roll_angle = radians_to_degrees(state.euler_angles[0]);
        msg.roll_angular_velocity = radians_to_degrees(state.angular_rate[0]);
        msg.roll_angle_setpoint = radians_to_degrees(setpoint.attitude[0]);
        msg.roll_inner_servo_tilt_angle = radians_to_degrees(output_.inner_servo_tilt_angle);

        msg.pitch_angle = radians_to_degrees(state.euler_angles[1]);
        msg.pitch_angular_velocity = radians_to_degrees(state.angular_rate[1]);
        msg.pitch_angle_setpoint = radians_to_degrees(setpoint.attitude[1]);
        msg.pitch_outer_servo_tilt_angle = radians_to_degrees(output_.outer_servo_tilt_angle);

        msg.yaw_angle = radians_to_degrees(state.euler_angles[2]);
        msg.yaw_angular_velocity = radians_to_degrees(state.angular_rate[2]);
        msg.yaw_angle_setpoint = radians_to_degrees(setpoint.attitude[2] + origin_yaw_);
        msg.yaw_delta_motor_pwm = radians_to_degrees(output_.delta_motor_pwm);
        
        attitude_controller_debug_publisher_->publish(msg);
    }

    void set_current_yaw_as_origin() {
        auto state = state_aggregator_->get_state();
        origin_yaw_ = state.euler_angles[2];

        RCLCPP_INFO(logger_, "Attitude controller origin yaw set to current yaw.");
        RCLCPP_INFO(logger_, "Yaw: %f", radians_to_degrees(origin_yaw_));
    }

private:
    std::vector<bool> controller_active_;

    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    Eigen::Vector3d k_p_;
    Eigen::Vector3d k_d_;
    Eigen::Vector3d k_i_;
    Eigen::Vector3d integrated_error_ = Eigen::Vector3d::Zero();
    double dt_;
    AttitudePIDControllerOutput output_;

    double origin_yaw_ = 0.0;

    rclcpp::Publisher<AttitudeControllerDebug>::SharedPtr attitude_controller_debug_publisher_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_ = rclcpp::get_logger("attitude_pid_controller");
};
