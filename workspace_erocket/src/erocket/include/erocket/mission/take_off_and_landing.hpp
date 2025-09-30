#pragma once
#include <rclcpp/rclcpp.hpp>
#include <erocket/constants.hpp>
#include <erocket/msg/setpoint_c5.hpp>
#include <erocket/controller/state.hpp>

using namespace erocket::msg;
using namespace erocket::constants::setpoint;
using Eigen::Vector3d;

struct Trajectory {
    Vector3d pd;
    Vector3d pd_dot;
    Vector3d pd_2dot;
    Vector3d pd_3dot;
    Vector3d pd_4dot;
};

class TakeOffSequence {
public:
    TakeOffSequence(
        rclcpp::Node* node,
        rclcpp::QoS qos,
        std::shared_ptr<StateAggregator> state_aggregator
    ) 
    : delta_t_{0.01}, takeoff_desired_height_{1.0}, takeoff_climb_duration_{5.0},
        hover_duration_{5.0}, landing_descent_duration_{5.0},
        state_aggregator_(state_aggregator),
        trajectory_setpoint_publisher_{node->create_publisher<SetpointC5>(
            CONTROLLER_INPUT_SETPOINT_C5_TOPIC, qos
        )},
        clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)}
    {
        takeoff_timer_ = node->create_wall_timer(
            std::chrono::duration<double>(delta_t_),
            std::bind(&TakeOffSequence::timer_callback, this)
        );
        // Prevent it from firing until explicitly started
        takeoff_timer_->cancel();
    }

    void start_takeoff() {
        takeoff_initial_state_ = state_aggregator_->get_state();
        takeoff_start_time_ = clock_->now();

        RCLCPP_INFO(
            rclcpp::get_logger("TakeOffSequence"), 
            "Starting takeoff sequence from height %.2f m to %.2f m",
            takeoff_initial_state_.position[2],
            takeoff_initial_state_.position[2] - takeoff_desired_height_
        );

        takeoff_timer_->reset();
    }

    // Optional external stop
    void abort() {
        takeoff_timer_->cancel();
    }

private:
    double delta_t_; // Time step for trajectory updates

    State takeoff_initial_state_;
    rclcpp::Time takeoff_start_time_;
    double takeoff_desired_height_;
    double takeoff_climb_duration_; // Duration to reach desired height

    double hover_duration_;  // Duration to hover at desired height

    State landing_initial_state_;
    rclcpp::Time landing_start_time_;
    double landing_descent_duration_; // Duration to land to ground

    std::shared_ptr<StateAggregator> state_aggregator_;
	rclcpp::Publisher<SetpointC5>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr takeoff_timer_;

    void timer_callback() {
        auto trajectory = takeoff_landing(
            clock_->now().seconds() - takeoff_start_time_.seconds(),
            state_aggregator_->get_state().position,
            takeoff_initial_state_.position,
            takeoff_desired_height_,
            takeoff_climb_duration_,
            landing_descent_duration_,
            takeoff_climb_duration_ + hover_duration_
        );

        SetpointC5 setpoint_msg;
        Eigen::Map<Eigen::Vector3d>(setpoint_msg.position.data()) = trajectory.pd;
        Eigen::Map<Eigen::Vector3d>(setpoint_msg.velocity.data()) = trajectory.pd_dot;
        Eigen::Map<Eigen::Vector3d>(setpoint_msg.acceleration.data()) = trajectory.pd_2dot;
        Eigen::Map<Eigen::Vector3d>(setpoint_msg.jerk.data()) = trajectory.pd_3dot;
        Eigen::Map<Eigen::Vector3d>(setpoint_msg.snap.data()) = trajectory.pd_4dot;
        setpoint_msg.yaw = takeoff_initial_state_.euler_angles[2];
        setpoint_msg.yaw_rate = 0.0;
        setpoint_msg.yaw_acceleration = 0.0;
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }

    Trajectory takeoff_landing(double t, 
                            const Vector3d& p, 
                            const Vector3d& p0, 
                            double hd, 
                            double ts, 
                            double td, 
                            double tL) 
    {
        static bool a = false;     // persistent variable
        static Vector3d p0L;       // landing start point

        Trajectory traj;

        if (t <= ts) {
            // Takeoff
            traj.pd << p0(0),
                    p0(1),
                    p0(2) - (hd / 2.0) * (1.0 - cos((t * M_PI) / ts));
            traj.pd_dot << 0.0,
                        0.0,
                        -(hd / 2.0) * (M_PI / ts) * sin((t * M_PI) / ts);
            traj.pd_2dot << 0.0,
                            0.0,
                            -((hd / 2.0) * pow(M_PI / ts, 2)) * cos((t * M_PI) / ts);
            traj.pd_3dot << 0.0,
                            0.0,
                            ((hd / 2.0) * pow(M_PI / ts, 3)) * sin((t * M_PI) / ts);
            traj.pd_4dot << 0.0,
                            0.0,
                            ((hd / 2.0) * pow(M_PI / ts, 4)) * cos((t * M_PI) / ts);

        } else if (t <= tL) {
            // Hover
            traj.pd << p0(0), p0(1), p0(2) - hd;
            traj.pd_dot.setZero();
            traj.pd_2dot.setZero();
            traj.pd_3dot.setZero();
            traj.pd_4dot.setZero();

        } else if (t <= tL + td) {
            // Landing
            if (!a) {
                p0L = p;
                a = true;
            }
            double dz = (p0L(2) - p0(2)) / 2.0;
            double tau = (t - tL) * M_PI / td;

            traj.pd << p0L(0),
                    p0L(1),
                    p0L(2) - dz * (1.0 - cos(tau));
            traj.pd_dot << 0.0,
                        0.0,
                        -dz * (M_PI / ts) * sin((t - tL) * M_PI / td); 
            traj.pd_2dot << 0.0,
                            0.0,
                            -(dz * pow(M_PI / ts, 2)) * cos((t - tL) * M_PI / td);
            traj.pd_3dot << 0.0,
                            0.0,
                            (dz * pow(M_PI / ts, 3)) * sin((t - tL) * M_PI / td);
            traj.pd_4dot << 0.0,
                            0.0,
                            (dz * pow(M_PI / ts, 4)) * cos((t - tL) * M_PI / td);

        } else {
            // Finished landing
            traj.pd << p0L(0), p0L(1), p0(2);
            traj.pd_dot.setZero();
            traj.pd_2dot.setZero();
            traj.pd_3dot.setZero();
            traj.pd_4dot.setZero();
        }

        return traj;
    }
};
