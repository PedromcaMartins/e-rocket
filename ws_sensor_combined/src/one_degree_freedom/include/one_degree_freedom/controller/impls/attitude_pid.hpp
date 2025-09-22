#pragma once
#include <rclcpp/rclcpp.hpp>

class AttitudePIDController
{
public:
    // updated asyncronously by the caller
    std::atomic<double> angle_;
    std::atomic<double> angular_rate;
    std::atomic<double> desired_setpoint_;
    // computed by the controller
    double tilt_angle_;

    AttitudePIDController(double k_p, double k_d, double k_i, double dt)
        : angle_{0.0f}, angular_rate{0.0f}, desired_setpoint_{0.0f}, tilt_angle_(0.0f), 
            k_p_(k_p), k_d_(k_d), k_i_(k_i), integrated_error_(0.0f), dt_(dt)
            {
                // Safety check
                if (std::isnan(k_p) || std::isnan(k_d) || std::isnan(k_i)) {
                    RCLCPP_ERROR(rclcpp::get_logger("attitude_pid_controller"), "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                // Safety check
                if (dt <= 0.0f || std::isnan(dt)) {
                    RCLCPP_ERROR(rclcpp::get_logger("attitude_pid_controller"), "Invalid time step provided.");
                    throw std::runtime_error("Time step invalid");
                }

                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_p: %f", k_p);
                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_d: %f", k_d);
                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_i: %f", k_i);
            }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    double compute() {
        double error_p = angle_ - desired_setpoint_;

        // Update the integrated error
        integrated_error_ += error_p * dt_; 

        // Compute control input
        double dot_product = error_p * k_p_ + angular_rate * k_d_;
        tilt_angle_ = dot_product + integrated_error_ * k_i_;
        return tilt_angle_;
    }

private:
    const double k_p_;
    const double k_d_;
    const double k_i_;
    double integrated_error_;
    const double dt_;
};
