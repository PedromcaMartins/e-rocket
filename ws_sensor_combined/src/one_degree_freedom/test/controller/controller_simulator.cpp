#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_output_tilt_angle.hpp>
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

        tilt_angle_subscriber_ = this->create_subscription<ControllerOutputTiltAngle>(
            CONTROLLER_OUTPUT_TILT_ANGLE_TOPIC, qos,
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
	rclcpp::Subscription<ControllerOutputTiltAngle>::SharedPtr  tilt_angle_subscriber_;

    //!< State variables - to be updated by the simulator
    float delta_theta_ = 0.0f; // pitch angle
    float delta_omega_ = 0.0f; // angular position

	//!< Auxiliary functions
    void publish_attitude(float attitude);
    void publish_angular_rate(float angular_rate);
    void controller_output_callback(const ControllerOutputTiltAngle::SharedPtr msg);
};

void ControllerSimulator::publish_attitude(float attitude)
{
    ControllerInputAttitude msg{};
    msg.stamp = this->get_clock()->now();
    msg.attitude = attitude;
    attitude_publisher_->publish(msg);
}

void ControllerSimulator::publish_angular_rate(float angular_rate)
{
    ControllerInputAngularRate msg{};
    msg.stamp = this->get_clock()->now();
    msg.angular_rate = angular_rate;
    angular_rate_publisher_->publish(msg);
}

void ControllerSimulator::controller_output_callback(const ControllerOutputTiltAngle::SharedPtr msg)
{
    float delta_gamma = msg->tilt_angle;
    float step = CONTROLLER_DT;

    float a = delta_gamma * M * L * G / J;
    delta_omega_ += a * step;
    delta_theta_ += delta_omega_ * step;

    // Publish the new state variables
    publish_attitude(delta_theta_);
    publish_angular_rate(delta_omega_);
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
