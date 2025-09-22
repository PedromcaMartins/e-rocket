#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/frame_transforms.h>

class PositionPIDController
{
public:
    // updated asyncronously by the caller
    std::array<std::atomic<double>, 3> position_;
    std::array<std::atomic<double>, 3> position_setpoint_;
    std::array<std::atomic<double>, 3> velocity_;
    std::array<std::atomic<double>, 3> velocity_setpoint_;
    std::array<std::atomic<double>, 3> acceleration_;
    std::array<std::atomic<double>, 3> acceleration_setpoint_;
    std::atomic<double> yaw_angle_setpoint_;
    // computed by the controller
    Eigen::Vector3d desired_acceleration_;
    Eigen::Vector3d desired_attitude_;
    double desired_thrust_;

    PositionPIDController(
        double mass, 
        double g, 
        const Eigen::Vector3d & k_p, 
        const Eigen::Vector3d & k_d, 
        const Eigen::Vector3d & k_i,
        const Eigen::Vector3d & kff,
        const Eigen::Vector3d & min_output,
        const Eigen::Vector3d & max_output,
        double dt
    )   : mass_(mass), g_(g), k_p_(k_p), k_d_(k_d), k_i_(k_i), kff_(kff),
            min_output_(min_output), max_output_(max_output),
            origin_position_{Eigen::Vector3d(0.0f, 0.0f, 0.0f)}, origin_yaw_{0.0f}, dt_(dt)
            {
                if (mass <= 0.0f || std::isnan(mass)) {
                    RCLCPP_ERROR(rclcpp::get_logger("position_pid_controller"), "Invalid mass provided.");
                    throw std::runtime_error("Mass invalid");
                }

                if (k_p.size() != 3 || k_d.size() != 3 || k_i.size() != 3 || kff.size() != 3 || 
                    min_output.size() != 3 || max_output.size() != 3 || 
                    std::isnan(k_p[0]) || std::isnan(k_p[1]) || std::isnan(k_p[2]) ||
                    std::isnan(k_d[0]) || std::isnan(k_d[1]) || std::isnan(k_d[2]) ||
                    std::isnan(k_i[0]) || std::isnan(k_i[1]) || std::isnan(k_i[2]) ||
                    std::isnan(kff[0]) || std::isnan(kff[1]) || std::isnan(kff[2]) ||
                    std::isnan(min_output[0]) || std::isnan(min_output[1]) || std::isnan(min_output[2]) ||
                    std::isnan(max_output[0]) || std::isnan(max_output[1]) || std::isnan(max_output[2])
                ) {
                    RCLCPP_ERROR(rclcpp::get_logger("position_pid_controller"), "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                if (dt <= 0.0f || std::isnan(dt)) {
                    RCLCPP_ERROR(rclcpp::get_logger("position_pid_controller"), "Invalid time step provided.");
                    throw std::runtime_error("Time step invalid");
                }

                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "mass: %f", mass_);
                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "controller dt: %f", dt_);

                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "gains k_p: [%f, %f, %f]", k_p[0], k_p[1], k_p[2]);
                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "gains k_d: [%f, %f, %f]", k_d[0], k_d[1], k_d[2]);
                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "gains k_i: [%f, %f, %f]", k_i[0], k_i[1], k_i[2]);

                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "gains kff: [%f, %f, %f]", kff[0], kff[1], kff[2]);
                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "min output: [%f, %f, %f]", min_output[0], min_output[1], min_output[2]);
                RCLCPP_INFO(rclcpp::get_logger("position_pid_controller"), "max output: [%f, %f, %f]", max_output[0], max_output[1], max_output[2]);

                for (auto& val : position_) val = 0.0f;
                for (auto& val : position_setpoint_) val = 0.0f;
                for (auto& val : velocity_) val = 0.0f;
                for (auto& val : velocity_setpoint_) val = 0.0f;
                for (auto& val : acceleration_) val = 0.0f;
                for (auto& val : acceleration_setpoint_) val = 0.0f;

                yaw_angle_setpoint_ = 0.0f;
                desired_acceleration_.fill(0.0f);
                desired_attitude_.fill(0.0f);
                desired_thrust_ = 0.0f;

                error_i_.fill(0.0f);
            }

    void compute() {
        Eigen::Vector3d position, position_setpoint, velocity, velocity_setpoint, feed_forward_ref;
        for (size_t i = 0; i < 3; ++i) {
            position[i]          = position_[i] - origin_position_[i];
            position_setpoint[i] = position_setpoint_[i];
            velocity[i]          = velocity_[i];
            velocity_setpoint[i] = velocity_setpoint_[i];
            feed_forward_ref[i]  = acceleration_setpoint_[i];
        }

        // Compute the position error and velocity error using the path desired position and velocity
        Eigen::Vector3d error_p = position_setpoint - position;
        Eigen::Vector3d error_d = velocity_setpoint - velocity;

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
        desired_acceleration_[2] -= g_;

        double yaw = yaw_angle_setpoint_;
        Eigen::Matrix3d RzT;
        Eigen::Vector3d r3d;

        /* Compute the normalized thrust and r3d vector */
        desired_thrust_ = mass_ * desired_acceleration_.norm();

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

    void set_position_as_origin(Eigen::Vector3d position, double yaw) {
        for (size_t i = 0; i < 3; ++i) {
            position_setpoint_[i] = position[i];
        }
        yaw_angle_setpoint_ = yaw;

        origin_position_ = position;
        origin_yaw_ = yaw;
    }

private: 
    double mass_;
    double g_;
    Eigen::Vector3d k_p_;
    Eigen::Vector3d k_d_;
    Eigen::Vector3d k_i_;
    Eigen::Vector3d kff_;
    Eigen::Vector3d min_output_;
    Eigen::Vector3d max_output_;
    Eigen::Vector3d error_i_;

    Eigen::Vector3d origin_position_;
    double origin_yaw_;

    const double dt_;
};
