#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;

/**
 * @brief Node that simulates the effects of the controller on the environment. It is used for testing the controller
 */
class ControllerSimulator : public rclcpp::Node
{
public:
	explicit ControllerSimulator() : Node("controller_simulator")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        servo_tilt_angle_subscriber_ = this->create_subscription<ControllerOutputServoTiltAngle>(
            CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos,
            std::bind(&ControllerSimulator::controller_output_callback, this, std::placeholders::_1)
        );

        attitude_publisher_ = this->create_publisher<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos
        );

        angular_rate_publisher_ = this->create_publisher<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos
        );
	}

private:
	//!< Publishers and Subscribers
	rclcpp::Publisher<ControllerInputAttitude>::SharedPtr       attitude_publisher_;
	rclcpp::Publisher<ControllerInputAngularRate>::SharedPtr    angular_rate_publisher_;
	rclcpp::Subscription<ControllerOutputServoTiltAngle>::SharedPtr  servo_tilt_angle_subscriber_;

    //!< State variables - to be updated by the simulator
    float roll_delta_theta_ = 0.0f; // pitch angle
    float roll_delta_omega_ = 0.0f; // angular position
    float pitch_delta_theta_ = 0.0f; // pitch angle
    float pitch_delta_omega_ = 0.0f; // angular position

	//!< Auxiliary functions
    void publish_attitude();
    void publish_angular_rate();
    void controller_output_callback(const ControllerOutputServoTiltAngle::SharedPtr msg);
};

void ControllerSimulator::publish_attitude()
{
    ControllerInputAttitude msg{};
    msg.stamp = this->get_clock()->now();
    msg.roll_radians = roll_delta_theta_;
    msg.pitch_radians = pitch_delta_theta_;
    attitude_publisher_->publish(msg);
}

void ControllerSimulator::publish_angular_rate()
{
    ControllerInputAngularRate msg{};
    msg.stamp = this->get_clock()->now();
    msg.x_roll_angular_rate_radians_per_second = roll_delta_omega_;
    msg.y_pitch_angular_rate_radians_per_second = pitch_delta_omega_;
    angular_rate_publisher_->publish(msg);
}

void ControllerSimulator::controller_output_callback(const ControllerOutputServoTiltAngle::SharedPtr msg)
{
    float roll_delta_gamma = msg->inner_servo_tilt_angle_radians;
    float pitch_delta_gamma = msg->outer_servo_tilt_angle_radians;
    float step = CONTROLLER_DT_SECONDS;

    //*********//
    //* roll  *//
    //*********//
    {
        float a = roll_delta_gamma * M * L * G / J;
        roll_delta_omega_ += a * step;
        roll_delta_theta_ += roll_delta_omega_ * step;
    }

    //*********//
    //* pitch *//
    //*********//
    {
        float a = pitch_delta_gamma * M * L * G / J;
        pitch_delta_omega_ += a * step;
        pitch_delta_theta_ += pitch_delta_omega_ * step;
    }

    // Publish the new state variables
    publish_attitude();
    publish_angular_rate();
}

int main(int argc, char *argv[])
{
	std::cout << "Starting controller simulator..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControllerSimulator>());

	rclcpp::shutdown();
	return 0;
}
