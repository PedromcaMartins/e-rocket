#include <rclcpp/rclcpp.hpp>

#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::msg;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2MessageMappingTest : public rclcpp::Node
{
public: 
    Px4Ros2MessageMappingTest() : 
		Node("px4_ros2_flight_mode_test"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
        attitude_radians_{0.0f},
        angular_rate_radians_{0.0f},
        controller_output_servo_tilt_angle_publisher_{this->create_publisher<ControllerOutputServoTiltAngle>(
            CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_
        )},
        controller_output_motor_thrust_publisher_{this->create_publisher<ControllerOutputMotorThrust>(
            CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_
        )},
        controller_input_attitude_subscription_{this->create_subscription<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_, 
            [this](const ControllerInputAttitude::SharedPtr msg) {
                attitude_radians_.store(msg->attitude_radians);
            }
        )},
        controller_input_angular_rate_subscription_{this->create_subscription<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_, 
            [this](const ControllerInputAngularRate::SharedPtr msg) {
                angular_rate_radians_.store(msg->angular_rate_radians_per_second);
            }
        )}
    {
        test_timer_ = this->create_wall_timer(100ms,
            [this]() {
                static double time = 0.0;
                float sin_wave = 0.05f * (sin(time / 10.0f)); // Sinusoidal wave between 0 and 0.10
                time++;

                if (sin_wave < 0.0f) {
                    sin_wave = NAN;
                }

                publish_controller_output_servo_tilt_angle_(sin_wave);
                publish_controller_output_motor_thrust_(sin_wave, sin_wave);
                RCLCPP_INFO(this->get_logger(), "attitude radians: %f", attitude_radians_.load());
                RCLCPP_INFO(this->get_logger(), "angular rate radians: %f", angular_rate_radians_.load());
            }
		);
    }

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	std::atomic<float> attitude_radians_;
	std::atomic<float> angular_rate_radians_;

    rclcpp::Publisher<ControllerOutputServoTiltAngle>::SharedPtr controller_output_servo_tilt_angle_publisher_;
    rclcpp::Publisher<ControllerOutputMotorThrust>::SharedPtr controller_output_motor_thrust_publisher_;
    rclcpp::Subscription<ControllerInputAttitude>::SharedPtr controller_input_attitude_subscription_;
    rclcpp::Subscription<ControllerInputAngularRate>::SharedPtr controller_input_angular_rate_subscription_;

	rclcpp::TimerBase::SharedPtr test_timer_;

    void publish_controller_output_servo_tilt_angle_(float servo_tilt_angle_radians);
    void publish_controller_output_motor_thrust_(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage);
};

void Px4Ros2MessageMappingTest::publish_controller_output_servo_tilt_angle_(float servo_tilt_angle_radians) {
    ControllerOutputServoTiltAngle msg {};
    msg.servo_tilt_angle_radians = servo_tilt_angle_radians;
    msg.stamp = this->get_clock()->now();
    controller_output_servo_tilt_angle_publisher_->publish(msg);
}

void Px4Ros2MessageMappingTest::publish_controller_output_motor_thrust_(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage) {
    ControllerOutputMotorThrust msg {};
    msg.upwards_motor_thrust_percentage = upwards_motor_thrust_percentage;
    msg.downwards_motor_thrust_percentage = downwards_motor_thrust_percentage;
    msg.stamp = this->get_clock()->now();
    controller_output_motor_thrust_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Message Mapping Test node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2MessageMappingTest>());

	rclcpp::shutdown();
	return 0;
}
