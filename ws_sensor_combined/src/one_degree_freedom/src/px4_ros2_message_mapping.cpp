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
#include <one_degree_freedom/frame_transforms.h>

#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::frame_transforms::utils::quaternion;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2MessageMapping : public rclcpp::Node
{
public: 
    Px4Ros2MessageMapping() : 
		Node("px4_ros2_message_mapping"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

        actuator_servos_publisher_{this->create_publisher<px4_msgs::msg::ActuatorServos>(
            "/fmu/in/actuator_servos", qos_
        )},
        controller_output_servo_tilt_angle_subscription_{this->create_subscription<one_degree_freedom::msg::ControllerOutputServoTiltAngle>(
            CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_,
            std::bind(&Px4Ros2MessageMapping::controller_output_servo_tilt_angle_callback, this, std::placeholders::_1)
        )},

		actuator_motors_publisher_{this->create_publisher<px4_msgs::msg::ActuatorMotors>(
            "/fmu/in/actuator_motors", qos_
        )},
        controller_output_motor_thrust_subscription_{this->create_subscription<one_degree_freedom::msg::ControllerOutputMotorThrust>(
            CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_,
            std::bind(&Px4Ros2MessageMapping::controller_output_motor_thrust_callback, this, std::placeholders::_1)
        )},

		vehicle_attitude_subscription_{this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos_,
            std::bind(&Px4Ros2MessageMapping::vehicle_attitude_callback, this, std::placeholders::_1)
        )},
        controller_input_attitude_publisher_{this->create_publisher<one_degree_freedom::msg::ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_
        )},

        vehicle_angular_velocity_subscription_{this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos_,
            std::bind(&Px4Ros2MessageMapping::vehicle_angular_velocity_callback, this, std::placeholders::_1)
        )},
        controller_input_angular_rate_publisher_{this->create_publisher<one_degree_freedom::msg::ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_
        )}
    {
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

float servo_tilt_angle_radians_to_servo_pwm(float servo_tilt_angle_radians) {
    auto servo_pwm = servo_tilt_angle_radians;

    if (servo_pwm > 1.0f) {
        servo_pwm = 1.0f;
    } else if (servo_pwm < -1.0f) {
        servo_pwm = -1.0f;
    }

    return servo_pwm;
}

void Px4Ros2MessageMapping::controller_output_servo_tilt_angle_callback(const one_degree_freedom::msg::ControllerOutputServoTiltAngle::SharedPtr ros2_msg) {
    auto servo_tilt_angle_radians = ros2_msg->servo_tilt_angle_radians;
    auto servo_pwm = servo_tilt_angle_radians_to_servo_pwm(servo_tilt_angle_radians);

    px4_msgs::msg::ActuatorServos px4_msg {};
    px4_msg.control[0] = servo_pwm;
	px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_servos_publisher_->publish(px4_msg);
}

void Px4Ros2MessageMapping::controller_output_motor_thrust_callback(const one_degree_freedom::msg::ControllerOutputMotorThrust::SharedPtr ros2_msg) {
    px4_msgs::msg::ActuatorMotors px4_msg {};
    px4_msg.control[0] = ros2_msg->upwards_motor_thrust_percentage;
    px4_msg.control[0] = ros2_msg->downwards_motor_thrust_percentage;
	px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    actuator_motors_publisher_->publish(px4_msg);
}

void Px4Ros2MessageMapping::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr px4_msg) {
    double roll, pitch, yaw;
    const auto &q = px4_msg->q;
    Eigen::Quaterniond quaternion(q[0], q[1], q[2], q[3]);
    quaternion_to_euler(quaternion, roll, pitch, yaw);

    (void) roll;
    (void) yaw;

    one_degree_freedom::msg::ControllerInputAttitude ros2_msg {};
    ros2_msg.attitude_radians = pitch;
    ros2_msg.stamp = this->get_clock()->now();
    controller_input_attitude_publisher_->publish(ros2_msg);
}

void Px4Ros2MessageMapping::vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr px4_msg) {
    double _roll_angular_rate = px4_msg->xyz[0];
    (void) _roll_angular_rate;
    
    double pitch_angular_rate = px4_msg->xyz[1];

    double _yaw_angular_rate = px4_msg->xyz[2];
    (void) _yaw_angular_rate;

    one_degree_freedom::msg::ControllerInputAngularRate ros2_msg {};
    ros2_msg.angular_rate_radians_per_second = pitch_angular_rate;
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
