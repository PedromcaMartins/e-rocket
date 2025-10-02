#pragma once
#include <rclcpp/rclcpp.hpp>

using Eigen::Vector3d;

struct Trajectory {
    Vector3d pd;       // position
    Vector3d pd_dot;   // velocity
    Vector3d pd_2dot;  // acceleration
    Vector3d pd_3dot;  // jerk
    Vector3d pd_4dot;  // snap
};

// -----------------------------------
// Compute takeoff + hover trajectory
// -----------------------------------
inline Trajectory compute_takeoff(
    double t,                  // current time [s] since start of takeoff
    const Vector3d& p0,        // initial position (start of takeoff)
    double hd,                 // desired climb height
    double ts                  // takeoff climb duration
) {
    Trajectory traj;

    if (t <= ts) {
        // Takeoff: smooth cosine climb
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

    } else {
        // Stay hovering if t > ts+hover
        traj.pd << p0(0), p0(1), p0(2) - hd;
        traj.pd_dot.setZero();
        traj.pd_2dot.setZero();
        traj.pd_3dot.setZero();
        traj.pd_4dot.setZero();
    }

    return traj;
}

// -----------------------------------
// Compute landing trajectory
// -----------------------------------
inline Trajectory compute_landing(
    double t,                  // current time [s] since start of landing
    const Vector3d& p0L,       // starting point of landing (hover pos)
    const Vector3d& ground_p0, // ground position reference
    double td                  // descent duration
) {
    Trajectory traj;

    if (t <= td) {
        // Smooth cosine descent
        double dz = (p0L(2) - ground_p0(2)) / 2.0;
        double tau = (t * M_PI) / td;

        traj.pd << p0L(0),
                    p0L(1),
                    p0L(2) - dz * (1.0 - cos(tau));
        traj.pd_dot << 0.0,
                        0.0,
                        -dz * (M_PI / td) * sin(tau);
        traj.pd_2dot << 0.0,
                        0.0,
                        -(dz * pow(M_PI / td, 2)) * cos(tau);
        traj.pd_3dot << 0.0,
                        0.0,
                        (dz * pow(M_PI / td, 3)) * sin(tau);
        traj.pd_4dot << 0.0,
                        0.0,
                        (dz * pow(M_PI / td, 4)) * cos(tau);

    } else {
        // Finished landing (on the ground)
        traj.pd << p0L(0), p0L(1), ground_p0(2);
        traj.pd_dot.setZero();
        traj.pd_2dot.setZero();
        traj.pd_3dot.setZero();
        traj.pd_4dot.setZero();
    }

    return traj;
}
