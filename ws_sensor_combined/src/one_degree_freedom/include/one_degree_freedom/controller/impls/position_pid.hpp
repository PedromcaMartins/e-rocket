#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/position_controller_debug.hpp>
#include <one_degree_freedom/frame_transforms.h>
#include <one_degree_freedom/vehicle_constants.hpp>

using namespace one_degree_freedom::msg;

namespace frame_transforms = one_degree_freedom::frame_transforms;

struct PositionPIDControllerOutput {
    Eigen::Vector3d desired_acceleration;
    Eigen::Vector3d desired_attitude;
    double desired_thrust;
};

class PositionPIDController
{
public:
    PositionPIDController(
        rclcpp::Node* node,
        rclcpp::QoS qos,
        std::shared_ptr<VehicleConstants> vehicle_constants,
        std::shared_ptr<StateAggregator> state_aggregator,
        std::shared_ptr<SetpointAggregator> setpoint_aggregator
    )   : vehicle_constants_(vehicle_constants), 
            state_aggregator_{state_aggregator}, setpoint_aggregator_{setpoint_aggregator},
            debug_publisher_{node->create_publisher<PositionControllerDebug>(
                CONTROLLER_POSITION_DEBUG_TOPIC, qos
            )},
            clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
            {
                node->declare_parameter<double>(CONTROLLER_POSITION_FREQUENCY_HERTZ_PARAM);
                double controllers_freq = node->get_parameter(CONTROLLER_POSITION_FREQUENCY_HERTZ_PARAM).as_double();
                if (controllers_freq <= 0.0f || std::isnan(controllers_freq)) {
                    RCLCPP_ERROR(node->get_logger(), "Could not read controller frequency correctly.");
                    throw std::runtime_error("Controller frequency invalid");
                }
                dt_ = 1.0 / controllers_freq;

                node->declare_parameter<bool>(CONTROLLER_POSITION_ACTIVE_PARAM);
                controller_active_ = node->get_parameter(CONTROLLER_POSITION_ACTIVE_PARAM).as_bool();
                RCLCPP_INFO(node->get_logger(), "Position controller: %s", (controller_active_)? "active" : "off");

                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_P_PARAM);
                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_D_PARAM);
                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_I_PARAM);
                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_FF_PARAM);
                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_MIN_OUTPUT_PARAM);
                node->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_MAX_OUTPUT_PARAM);
                k_p_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_K_P_PARAM).as_double_array().data(), 3);
                k_d_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_K_D_PARAM).as_double_array().data(), 3);
                k_i_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_K_I_PARAM).as_double_array().data(), 3);
                k_ff_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_K_FF_PARAM).as_double_array().data(), 3);
                min_output_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_MIN_OUTPUT_PARAM).as_double_array().data(), 3);
                max_output_ = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_POSITION_MAX_OUTPUT_PARAM).as_double_array().data(), 3);
                if (k_p_.size() != 3 || k_d_.size() != 3 || k_i_.size() != 3 || k_ff_.size() != 3 ||
                    min_output_.size() != 3 || max_output_.size() != 3 ||
                    std::isnan(k_p_[0]) || std::isnan(k_p_[1]) || std::isnan(k_p_[2]) ||
                    std::isnan(k_d_[0]) || std::isnan(k_d_[1]) || std::isnan(k_d_[2]) ||
                    std::isnan(k_i_[0]) || std::isnan(k_i_[1]) || std::isnan(k_i_[2]) ||
                    std::isnan(k_ff_[0]) || std::isnan(k_ff_[1]) || std::isnan(k_ff_[2]) ||
                    std::isnan(min_output_[0]) || std::isnan(min_output_[1]) || std::isnan(min_output_[2]) ||
                    std::isnan(max_output_[0]) || std::isnan(max_output_[1]) || std::isnan(max_output_[2])
                ) {
                    RCLCPP_ERROR(logger_, "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                RCLCPP_INFO(logger_, "controller dt: %f", dt_);

                RCLCPP_INFO(logger_, "gains k_p: [%f, %f, %f]", k_p_[0], k_p_[1], k_p_[2]);
                RCLCPP_INFO(logger_, "gains k_d: [%f, %f, %f]", k_d_[0], k_d_[1], k_d_[2]);
                RCLCPP_INFO(logger_, "gains k_i: [%f, %f, %f]", k_i_[0], k_i_[1], k_i_[2]);

                RCLCPP_INFO(logger_, "gains k_ff: [%f, %f, %f]", k_ff_[0], k_ff_[1], k_ff_[2]);
                RCLCPP_INFO(logger_, "min output: [%f, %f, %f]", min_output_[0], min_output_[1], min_output_[2]);
                RCLCPP_INFO(logger_, "max output: [%f, %f, %f]", max_output_[0], max_output_[1], max_output_[2]);
            }

    PositionPIDControllerOutput compute() {
        auto state = state_aggregator_->get_state();
        Eigen::Vector3d position = state.position - origin_position_;
        Eigen::Vector3d velocity = state.velocity;

        auto setpoint = setpoint_aggregator_->get_position_setpoint();
        auto feed_forward_ref  = setpoint.acceleration;

        Eigen::Vector3d desired_acceleration, desired_attitude;
        double desired_thrust;

        // Compute the position error and velocity error using the path desired position and velocity
        Eigen::Vector3d error_p = setpoint.position - position;
        Eigen::Vector3d error_d = setpoint.velocity - velocity;

        // Compute the desired control output acceleration for each controller
        // Compute the integral term (using euler integration) - TODO: improve the integration part
        error_i_ += k_i_.cwiseProduct(error_p * dt_);

        // // Compute the PID terms
        Eigen::Vector3d p_term = k_p_.cwiseProduct(error_p);
        Eigen::Vector3d d_term = k_d_.cwiseProduct(error_d);
        Eigen::Vector3d i_term = error_i_;
        Eigen::Vector3d ff_term = k_ff_.cwiseProduct(feed_forward_ref);

        // // Compute the output and saturate it
        Eigen::Vector3d output = p_term + d_term + i_term + ff_term;
        for (size_t i = 0; i < 3; ++i) {
            desired_acceleration[i] = std::max(min_output_[i], std::min(output[i], max_output_[i]));
        }
        desired_acceleration[2] -= vehicle_constants_->gravitational_acceleration_;

        double yaw = setpoint.yaw;
        Eigen::Matrix3d RzT;
        Eigen::Vector3d r3d;

        /* Compute the normalized thrust and r3d vector */
        desired_thrust = vehicle_constants_->mass_of_system_ * desired_acceleration.norm();

        /* Compute the rotation matrix about the Z-axis */
        RzT << cos(yaw), sin(yaw), 0.0,
            -sin(yaw), cos(yaw), 0.0,
                    0.0,      0.0, 1.0;

        /* Compute the normalized rotation */
        r3d = -RzT * desired_acceleration / desired_acceleration.norm();

        /* Compute the actual attitude and setup the desired thrust to apply to the vehicle */
        desired_attitude << asin(-r3d[1]), atan2(r3d[0], r3d[2]), yaw;

        /* Saturate attitude */
        for (size_t i = 0; i < 2; ++i) {
            desired_attitude[i] = std::max(-M_PI / 6., std::min(desired_attitude[i], M_PI / 6.));
        }

        output_ = PositionPIDControllerOutput{
            desired_acceleration,
            desired_attitude,
            desired_thrust
        };

        publish_position_controller_debug();

        return output_;
    }

    bool is_controller_active() const { return controller_active_; }

private: 
    bool controller_active_;

    Eigen::Vector3d k_p_;
    Eigen::Vector3d k_d_;
    Eigen::Vector3d k_i_;
    Eigen::Vector3d k_ff_;
    Eigen::Vector3d min_output_;
    Eigen::Vector3d max_output_;
    Eigen::Vector3d error_i_ = Eigen::Vector3d::Zero();
    double dt_;
    PositionPIDControllerOutput output_;

    // origin position
    Eigen::Vector3d origin_position_ = Eigen::Vector3d(0.0f, 0.0f, 0.0f);

    std::shared_ptr<VehicleConstants> vehicle_constants_;
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;

    rclcpp::Publisher<PositionControllerDebug>::SharedPtr debug_publisher_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_ = rclcpp::get_logger("PositionPIDController");

    void publish_position_controller_debug() {
        PositionControllerDebug msg{};
        msg.stamp = clock_->now();

        auto state = state_aggregator_->get_state();
        auto setpoint = setpoint_aggregator_->get_position_setpoint();

        Eigen::Map<Eigen::Vector3d>(msg.position.data()) = state.position;
        Eigen::Map<Eigen::Vector3d>(msg.position_setpoint.data()) = setpoint.position + origin_position_;
        Eigen::Map<Eigen::Vector3d>(msg.velocity.data()) = state.velocity;
        Eigen::Map<Eigen::Vector3d>(msg.velocity_setpoint.data()) = setpoint.velocity;
        Eigen::Map<Eigen::Vector3d>(msg.acceleration.data()) = state.acceleration;
        Eigen::Map<Eigen::Vector3d>(msg.acceleration_setpoint.data()) = setpoint.acceleration;

        msg.yaw_angle_setpoint = frame_transforms::radians_to_degrees(setpoint.yaw);

        Eigen::Map<Eigen::Vector3d>(msg.desired_acceleration.data()) = output_.desired_acceleration;
        msg.desired_attitude[0] = frame_transforms::radians_to_degrees(output_.desired_attitude[0]);
        msg.desired_attitude[1] = frame_transforms::radians_to_degrees(output_.desired_attitude[1]);
        msg.desired_attitude[2] = frame_transforms::radians_to_degrees(output_.desired_attitude[2]);
        msg.desired_thrust = output_.desired_thrust;

        debug_publisher_->publish(msg);
    }
};
