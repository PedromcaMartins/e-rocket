#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <erocket/frame_transforms.h>
#include <erocket/constants.hpp>

namespace frame_transforms = erocket::frame_transforms;
using namespace px4_msgs::msg;
using namespace erocket::constants::controller;

struct State {
    // original messages
    VehicleAttitude attitude;
    VehicleAngularVelocity angular_velocity;
    VehicleLocalPosition local_position;
    VehicleOdometry odometry;

    // derived states
    // attitude
    Eigen::Quaterniond quaternion;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d euler_angles;

    // angular rates
    Eigen::Vector3d angular_rate;

    // linear position
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};

class StateAggregator {
public:
    StateAggregator(rclcpp::Node* node, rclcpp::QoS qos) :
    attitude_sub_{node->create_subscription<VehicleAttitude>(
        CONTROLLER_INPUT_ATTITUDE_TOPIC, qos,
        [this](const VehicleAttitude::SharedPtr msg) {
            auto q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            this->state_.quaternion = q;
            this->state_.rotation_matrix = q.toRotationMatrix();

            auto euler_angles = frame_transforms::quaternion_to_euler_radians(q);
            this->state_.euler_angles = Eigen::Vector3d(
                euler_angles.roll, euler_angles.pitch, euler_angles.yaw
            );

            this->state_.attitude = *msg;
        }
    )},
    angular_rate_sub_{node->create_subscription<VehicleAngularVelocity>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos,
        [this](const VehicleAngularVelocity::SharedPtr msg) {
            this->state_.angular_rate = Eigen::Vector3d(
                msg->xyz[0], msg->xyz[1], msg->xyz[2]
            );
            this->state_.angular_velocity = *msg;
        }
    )},
    local_position_sub_{node->create_subscription<VehicleLocalPosition>(
        CONTROLLER_INPUT_LOCAL_POSITION_TOPIC, qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            this->state_.position = Eigen::Vector3d(msg->x, msg->y, msg->z);
            this->state_.velocity = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);
            this->state_.acceleration = Eigen::Vector3d(msg->ax, msg->ay, msg->az);

            this->state_.local_position = *msg;
        }
    )},
    odometry_sub_{node->create_subscription<VehicleOdometry>(
        CONTROLLER_INPUT_ODOMETRY_TOPIC, qos,
        [this](const VehicleOdometry::SharedPtr msg) {
            this->state_.odometry = *msg;
        }
    )} { }

    const State get_state() const { return state_; }

private:
    State state_;

	rclcpp::Subscription<VehicleAttitude>::SharedPtr        attitude_sub_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr angular_rate_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr   local_position_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr        odometry_sub_;
};
