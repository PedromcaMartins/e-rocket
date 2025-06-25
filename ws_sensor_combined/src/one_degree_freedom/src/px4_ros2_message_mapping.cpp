#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_servos.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>

#include <one_degree_freedom/constants.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::px4_ros2_message_mapping;

struct Euler {
    float roll, pitch, yaw;
};

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
        this->declare_parameter<bool>(SERVOS_MAPPING_PARAM, true);
        this->declare_parameter<bool>(MOTORS_MAPPING_PARAM, true);
        this->declare_parameter<bool>(ATTITUDE_MAPPING_PARAM, true);
        this->declare_parameter<bool>(ANGULAR_RATE_MAPPING_PARAM, true);

        // get values from config
        bool servos_mapping = this->get_parameter(SERVOS_MAPPING_PARAM).as_bool();
        bool motors_mapping = this->get_parameter(MOTORS_MAPPING_PARAM).as_bool();
        bool attitude_mapping = this->get_parameter(ATTITUDE_MAPPING_PARAM).as_bool();
        bool angular_rate_mapping = this->get_parameter(ANGULAR_RATE_MAPPING_PARAM).as_bool();

        RCLCPP_INFO(this->get_logger(), "Servos Mapping %s", (servos_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Motors Mapping %s", (motors_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Attitude Mapping %s", (attitude_mapping)? "Activated" : "Deactivated");
        RCLCPP_INFO(this->get_logger(), "Angular Rate Mapping %s", (angular_rate_mapping)? "Activated" : "Deactivated");

        if (servos_mapping) {
            actuator_servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>(
                "/fmu/in/actuator_servos", qos_
            );
            controller_output_servo_tilt_angle_subscription_ = this->create_subscription<one_degree_freedom::msg::ControllerOutputServoTiltAngle>(
                CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_,
                std::bind(&Px4Ros2MessageMapping::controller_output_servo_tilt_angle_callback, this, std::placeholders::_1)
            );
        }

        if (motors_mapping) {
            actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
                "/fmu/in/actuator_motors", qos_
            );
            controller_output_motor_thrust_subscription_ = this->create_subscription<one_degree_freedom::msg::ControllerOutputMotorThrust>(
                CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_,
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
 * @brief Convert quaternion to Euler angles (radiands)
 * @param q Quaternion
 * @return Euler angles (roll, pitch, yaw)
 */
Euler quaternionToEulerRadians(const Eigen::Quaternionf q) {
    auto w = q.w();
    auto x = q.x();
    auto y = q.y();
    auto z = q.z();

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);
    float pitch = 0.0;

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(90.0, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
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
    auto q = Eigen::Quaternionf(px4_msg->q[0], px4_msg->q[1], px4_msg->q[2], px4_msg->q[3]);
    auto euler = quaternionToEulerRadians(q);

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

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Message Mapping node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2MessageMapping>());

	rclcpp::shutdown();
	return 0;
}
