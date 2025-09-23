#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/frame_transforms.h>
#include <one_degree_freedom/vehicle_constants.hpp>

class PositionPIDController
{
public:
    // computed by the controller
    Eigen::Vector3d desired_acceleration_;
    Eigen::Vector3d desired_attitude_;
    double desired_thrust_;

    PositionPIDController(
        std::shared_ptr<VehicleConstants> vehicle_constants,
        const Eigen::Vector3d & k_p, 
        const Eigen::Vector3d & k_d, 
        const Eigen::Vector3d & k_i,
        const Eigen::Vector3d & kff,
        const Eigen::Vector3d & min_output,
        const Eigen::Vector3d & max_output,
        const double dt,
        std::shared_ptr<StateAggregator> state_aggregator,
        std::shared_ptr<SetpointAggregator> setpoint_aggregator
    )   : vehicle_constants_(vehicle_constants), 
            k_p_(k_p), k_d_(k_d), k_i_(k_i), kff_(kff),
            min_output_(min_output), max_output_(max_output), dt_(dt), 
            origin_position_{Eigen::Vector3d(0.0f, 0.0f, 0.0f)}, origin_yaw_{0.0f}, 
            state_aggregator_{state_aggregator}, setpoint_aggregator_{setpoint_aggregator}
            {
                if (k_p.size() != 3 || k_d.size() != 3 || k_i.size() != 3 || kff.size() != 3 || 
                    min_output.size() != 3 || max_output.size() != 3 || 
                    std::isnan(k_p[0]) || std::isnan(k_p[1]) || std::isnan(k_p[2]) ||
                    std::isnan(k_d[0]) || std::isnan(k_d[1]) || std::isnan(k_d[2]) ||
                    std::isnan(k_i[0]) || std::isnan(k_i[1]) || std::isnan(k_i[2]) ||
                    std::isnan(kff[0]) || std::isnan(kff[1]) || std::isnan(kff[2]) ||
                    std::isnan(min_output[0]) || std::isnan(min_output[1]) || std::isnan(min_output[2]) ||
                    std::isnan(max_output[0]) || std::isnan(max_output[1]) || std::isnan(max_output[2])
                ) {
                    RCLCPP_ERROR(_logger, "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                if (dt <= 0.0f || std::isnan(dt)) {
                    RCLCPP_ERROR(_logger, "Invalid time step provided.");
                    throw std::runtime_error("Time step invalid");
                }

                RCLCPP_INFO(_logger, "controller dt: %f", dt_);

                RCLCPP_INFO(_logger, "gains k_p: [%f, %f, %f]", k_p[0], k_p[1], k_p[2]);
                RCLCPP_INFO(_logger, "gains k_d: [%f, %f, %f]", k_d[0], k_d[1], k_d[2]);
                RCLCPP_INFO(_logger, "gains k_i: [%f, %f, %f]", k_i[0], k_i[1], k_i[2]);

                RCLCPP_INFO(_logger, "gains kff: [%f, %f, %f]", kff[0], kff[1], kff[2]);
                RCLCPP_INFO(_logger, "min output: [%f, %f, %f]", min_output[0], min_output[1], min_output[2]);
                RCLCPP_INFO(_logger, "max output: [%f, %f, %f]", max_output[0], max_output[1], max_output[2]);

                desired_acceleration_.fill(0.0f);
                desired_attitude_.fill(0.0f);
                desired_thrust_ = 0.0f;

                error_i_.fill(0.0f);
            }

    void compute() {
        auto state = state_aggregator_->get_state();
        Eigen::Vector3d position = state.position;
        Eigen::Vector3d velocity = state.velocity;

        auto setpoint = setpoint_aggregator_->get_position_setpoint();
        auto feed_forward_ref  = setpoint.acceleration;

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
        Eigen::Vector3d ff_term = kff_.cwiseProduct(feed_forward_ref);

        // // Compute the output and saturate it
        Eigen::Vector3d output = p_term + d_term + i_term + ff_term;
        for (size_t i = 0; i < 3; ++i) {
            desired_acceleration_[i] = std::max(min_output_[i], std::min(output[i], max_output_[i]));
        }
        desired_acceleration_[2] -= vehicle_constants_->gravitational_acceleration_;

        double yaw = setpoint.yaw;
        Eigen::Matrix3d RzT;
        Eigen::Vector3d r3d;

        /* Compute the normalized thrust and r3d vector */
        desired_thrust_ = vehicle_constants_->mass_of_system_ * desired_acceleration_.norm();

        /* Compute the rotation matrix about the Z-axis */
        RzT << cos(yaw), sin(yaw), 0.0,
            -sin(yaw), cos(yaw), 0.0,
                    0.0,      0.0, 1.0;

        /* Compute the normalized rotation */
        r3d = -RzT * desired_acceleration_ / desired_acceleration_.norm();

        /* Compute the actual attitude and setup the desired thrust to apply to the vehicle */
        desired_attitude_ << asin(-r3d[1]), atan2(r3d[0], r3d[2]), yaw;

        /* Saturate attitude */
        for (size_t i = 0; i < 2; ++i) {
            desired_attitude_[i] = std::max(-M_PI / 6., std::min(desired_attitude_[i], M_PI / 6.));
        }
    }

    void set_current_position_as_origin() {
        auto state = state_aggregator_->get_state();
        origin_position_ = state.position;
        origin_yaw_ = state.euler_angles[2];

        RCLCPP_INFO(_logger, "Position controller origin set to current position.");
        RCLCPP_INFO(_logger, "Position: [%f, %f, %f], yaw: %f", 
            origin_position_[0], origin_position_[1], origin_position_[2], 
            origin_yaw_
        );
    }

private: 
    std::shared_ptr<VehicleConstants> vehicle_constants_;
    const Eigen::Vector3d k_p_;
    const Eigen::Vector3d k_d_;
    const Eigen::Vector3d k_i_;
    const Eigen::Vector3d kff_;
    const Eigen::Vector3d min_output_;
    const Eigen::Vector3d max_output_;
    const double dt_;
    Eigen::Vector3d error_i_;

    Eigen::Vector3d origin_position_;
    double origin_yaw_;

    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;

    rclcpp::Logger _logger = rclcpp::get_logger("PositionPIDController");
};
