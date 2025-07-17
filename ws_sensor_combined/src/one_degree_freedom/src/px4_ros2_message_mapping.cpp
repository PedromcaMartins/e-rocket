#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_servos.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <one_degree_freedom/constants.hpp>

#include <one_degree_freedom/frame_transforms.h>
#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::frame_transforms;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::px4_ros2_message_mapping;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2MessageMapping : public rclcpp::Node
{
public: 
    Px4Ros2MessageMapping() : 
		Node("px4_ros2_message_mapping"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)}
    {
        // declare parameters
        this->declare_parameter<bool>(MOCAP_MAPPING_PARAM, true);
        this->declare_parameter<bool>(SERVOS_MAPPING_PARAM, true);
        this->declare_parameter<bool>(MOTORS_MAPPING_PARAM, true);
        this->declare_parameter<bool>(ATTITUDE_MAPPING_PARAM, true);
        this->declare_parameter<bool>(ANGULAR_RATE_MAPPING_PARAM, true);

        // get values from config
        bool mocap_mapping = this->get_parameter(MOCAP_MAPPING_PARAM).as_bool();
        bool servos_mapping = this->get_parameter(SERVOS_MAPPING_PARAM).as_bool();
        bool motors_mapping = this->get_parameter(MOTORS_MAPPING_PARAM).as_bool();
        bool attitude_mapping = this->get_parameter(ATTITUDE_MAPPING_PARAM).as_bool();
        bool angular_rate_mapping = this->get_parameter(ANGULAR_RATE_MAPPING_PARAM).as_bool();

        RCLCPP_INFO(this->get_logger(), "MoCap Mapping %s", (mocap_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Servos Mapping %s", (servos_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Motors Mapping %s", (motors_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Attitude Mapping %s", (attitude_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Angular Rate Mapping %s", (angular_rate_mapping)? "Activated" : "Deactivated");

        if (mocap_mapping) {
            vehicle_mocap_pose_ned_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_mocap_odometry", qos_
            );
            mocap_pose_enu_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                MOCAP_TOPIC, qos_,
                std::bind(&Px4Ros2MessageMapping::mocap_pose_callback, this, std::placeholders::_1)
            );

            mocap_pose_enu_euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
                "/offboard/mocap_pose_enu_debug", qos_
            );
            vehicle_mocap_pose_ned_euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
                "/offboard/mocap_px4_ned_debug", qos_
            );
        }

        if (servos_mapping) {
            actuator_servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>(
                "/fmu/in/actuator_servos", qos_
            );
            controller_output_servo_tilt_angle_subscription_ = this->create_subscription<one_degree_freedom::msg::ControllerOutputServoTiltAngle>(
                CONTROLLER_OUTPUT_SERVO_PWM_TOPIC, qos_,
                std::bind(&Px4Ros2MessageMapping::controller_output_servo_tilt_angle_callback, this, std::placeholders::_1)
            );
        }

        if (motors_mapping) {
            actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
                "/fmu/in/actuator_motors", qos_
            );
            controller_output_motor_thrust_subscription_ = this->create_subscription<one_degree_freedom::msg::ControllerOutputMotorThrust>(
                CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC, qos_,
                std::bind(&Px4Ros2MessageMapping::controller_output_motor_thrust_callback, this, std::placeholders::_1)
            );
        }

        if (attitude_mapping) {
            vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
                "/fmu/out/vehicle_attitude", qos_,
                std::bind(&Px4Ros2MessageMapping::vehicle_attitude_callback, this, std::placeholders::_1)
            );
            controller_input_attitude_publisher_ = this->create_publisher<one_degree_freedom::msg::ControllerInputAttitude>(
                CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_
            );
        }

        if (angular_rate_mapping) {
            vehicle_angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
                "/fmu/out/vehicle_angular_velocity", qos_,
                std::bind(&Px4Ros2MessageMapping::vehicle_angular_velocity_callback, this, std::placeholders::_1)
            );
            controller_input_angular_rate_publisher_ = this->create_publisher<one_degree_freedom::msg::ControllerInputAngularRate>(
                CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_
            );
        }
	}

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_mocap_pose_ned_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_enu_subscription_;
    void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr ros2_msg);

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mocap_pose_enu_euler_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vehicle_mocap_pose_ned_euler_publisher_;
    void publish_mocap_enu_euler_angles(const Eigen::Quaterniond &q);
    void publish_vehicle_mocap_ned_euler_angles(const Eigen::Quaterniond &q);

	rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_publisher_;
    rclcpp::Subscription<one_degree_freedom::msg::ControllerOutputServoTiltAngle>::SharedPtr controller_output_servo_tilt_angle_subscription_;
    void controller_output_servo_tilt_angle_callback(const one_degree_freedom::msg::ControllerOutputServoTiltAngle::SharedPtr ros2_msg);

	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Subscription<one_degree_freedom::msg::ControllerOutputMotorThrust>::SharedPtr controller_output_motor_thrust_subscription_;
    void controller_output_motor_thrust_callback(const one_degree_freedom::msg::ControllerOutputMotorThrust::SharedPtr ros2_msg);

	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Publisher<one_degree_freedom::msg::ControllerInputAttitude>::SharedPtr controller_input_attitude_publisher_;
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr px4_msg);

    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_subscription_;
    rclcpp::Publisher<one_degree_freedom::msg::ControllerInputAngularRate>::SharedPtr controller_input_angular_rate_publisher_;
    void vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr px4_msg);
};

/**
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via mavlink to the vehicle autopilot filter to merge
 * @param ros2_msg A message with the pose of the vehicle expressed in ENU
 */
 void Px4Ros2MessageMapping::mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr ros2_msg) {
    px4_msgs::msg::VehicleOdometry px4_msg {};
    px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Convert ENU to NED
    Eigen::Vector3d ros2_enu_position = Eigen::Vector3d(
        ros2_msg->pose.position.x,
        ros2_msg->pose.position.y,
        ros2_msg->pose.position.z
    );
    Eigen::Vector3d px4_ned_position = transform_static_frame(ros2_enu_position, StaticTF::ENU_TO_NED);
    px4_msg.position[0] = px4_ned_position.x();
    px4_msg.position[1] = px4_ned_position.y();
    px4_msg.position[2] = px4_ned_position.z();

    // Convert quaternion from ROS2 to PX4 format
    Eigen::Quaterniond ros2_enu_orientation = Eigen::Quaterniond(
        ros2_msg->pose.orientation.w,
        ros2_msg->pose.orientation.x,
        ros2_msg->pose.orientation.y,
        ros2_msg->pose.orientation.z
    );
    Eigen::Quaterniond intermediate_baselink_orientation = transform_orientation(ros2_enu_orientation, StaticTF::BASELINK_TO_AIRCRAFT);
    Eigen::Quaterniond px4_ned_orientation = transform_orientation(intermediate_baselink_orientation, StaticTF::ENU_TO_NED);
    px4_msg.q[0] = px4_ned_orientation.w();
    px4_msg.q[1] = px4_ned_orientation.x();
    px4_msg.q[2] = px4_ned_orientation.y();
    px4_msg.q[3] = px4_ned_orientation.z();

    // Mocap does not provide, so we set them to zero
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    px4_msg.velocity[0] = 0.0f;
    px4_msg.angular_velocity[0] = 0.0f;
    px4_msg.position_variance[0] = 0.0f;
    px4_msg.orientation_variance[0] = 0.0f;
    px4_msg.velocity_variance[0] = 0.0f;

    vehicle_mocap_pose_ned_publisher_->publish(px4_msg);

    publish_mocap_enu_euler_angles(ros2_enu_orientation);
    publish_vehicle_mocap_ned_euler_angles(px4_ned_orientation);
}

void Px4Ros2MessageMapping::controller_output_servo_tilt_angle_callback(const one_degree_freedom::msg::ControllerOutputServoTiltAngle::SharedPtr ros2_msg) {
    px4_msgs::msg::ActuatorServos px4_msg {};
    px4_msg.control[0] = ros2_msg->outer_servo_tilt_angle_radians;
    px4_msg.control[1] = ros2_msg->inner_servo_tilt_angle_radians;
	px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_servos_publisher_->publish(px4_msg);
}

void Px4Ros2MessageMapping::controller_output_motor_thrust_callback(const one_degree_freedom::msg::ControllerOutputMotorThrust::SharedPtr ros2_msg) {
    px4_msgs::msg::ActuatorMotors px4_msg {};
    px4_msg.control[0] = ros2_msg->downwards_motor_thrust_percentage;
    px4_msg.control[1] = ros2_msg->upwards_motor_thrust_percentage;
	px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    actuator_motors_publisher_->publish(px4_msg);
}

void Px4Ros2MessageMapping::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr px4_msg) {
    auto q = Eigen::Quaterniond(px4_msg->q[0], px4_msg->q[1], px4_msg->q[2], px4_msg->q[3]);
    EulerAngle euler = quaternion_to_euler_radians(q);

    one_degree_freedom::msg::ControllerInputAttitude ros2_msg {};
    ros2_msg.roll_radians = euler.roll;
    ros2_msg.pitch_radians = euler.pitch;
    ros2_msg.yaw_radians = euler.yaw;
    ros2_msg.stamp = this->get_clock()->now();
    controller_input_attitude_publisher_->publish(ros2_msg);
}

void Px4Ros2MessageMapping::vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr px4_msg) {
    one_degree_freedom::msg::ControllerInputAngularRate ros2_msg {};
    ros2_msg.x_roll_angular_rate_radians_per_second = px4_msg->xyz[0];
    ros2_msg.y_pitch_angular_rate_radians_per_second = px4_msg->xyz[1];
    ros2_msg.z_yaw_angular_rate_radians_per_second = px4_msg->xyz[2];
    ros2_msg.stamp = this->get_clock()->now();
    controller_input_angular_rate_publisher_->publish(ros2_msg);
}

void Px4Ros2MessageMapping::publish_mocap_enu_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_radians(q);
    auto to_degrees = [](float radians) {
        return radians * (180.0f / M_PI);
    };
    geometry_msgs::msg::Vector3 euler_msg;
    euler_msg.x = to_degrees(euler.roll);
    euler_msg.y = to_degrees(euler.pitch);
    euler_msg.z = to_degrees(euler.yaw);
    mocap_pose_enu_euler_publisher_->publish(euler_msg);
}

void Px4Ros2MessageMapping::publish_vehicle_mocap_ned_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_radians(q);
    auto to_degrees = [](float radians) {
        return radians * (180.0f / M_PI);
    };
    geometry_msgs::msg::Vector3 euler_msg;
    euler_msg.x = to_degrees(euler.roll);
    euler_msg.y = to_degrees(euler.pitch);
    euler_msg.z = to_degrees(euler.yaw);
    vehicle_mocap_pose_ned_euler_publisher_->publish(euler_msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Message Mapping node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2MessageMapping>());

	rclcpp::shutdown();
	return 0;
}
