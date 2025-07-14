#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::simulator;

/**
 * @brief Node that simulates the effects of the controller on the environment. It is used for testing the controller
 */
class Simulator : public rclcpp::Node
{
public:
	explicit Simulator() : Node("simulator")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        servo_tilt_angle_subscriber_ = this->create_subscription<ControllerOutputServoTiltAngle>(
            CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos,
            std::bind(&Simulator::controller_output_callback, this, std::placeholders::_1)
        );

        attitude_publisher_ = this->create_publisher<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos
        );

        angular_rate_publisher_ = this->create_publisher<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos
        );

        this->declare_parameter<float>(CONTROLLER_FREQUENCY_HERTZ_PARAM);
        auto controllers_freq = this->get_parameter(CONTROLLER_FREQUENCY_HERTZ_PARAM).as_double();
        time_step_seconds_ = 1.0 / controllers_freq;

        // Safety check
        if (time_step_seconds_ <= 0.0f || time_step_seconds_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read controller time step correctly.");
            throw std::runtime_error("Time step invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Controller period: %f seconds", time_step_seconds_);

        this->declare_parameter<float>(MASS_OF_SYSTEM);
        this->declare_parameter<float>(LENGTH_OF_PENDULUM);
        this->declare_parameter<float>(GRAVITATIONAL_ACCELERATION);
        this->declare_parameter<float>(MOMENT_OF_INERTIA);

        m_ = this->get_parameter(MASS_OF_SYSTEM).as_double();
        l_ = this->get_parameter(LENGTH_OF_PENDULUM).as_double();
        g_ = this->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        j_ = this->get_parameter(MOMENT_OF_INERTIA).as_double();

        // Safety check
        if (m_ <= 0.0f || l_ <= 0.0f || g_ <= 0.0f || j_ <= 0.0f || m_ == NAN || l_ == NAN || g_ == NAN || j_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read simulator parameters correctly.");
            throw std::runtime_error("Simulator parameters invalid");
        }

        RCLCPP_INFO(this->get_logger(), "Simulator parameters: m = %f, l = %f, g = %f, j = %f", m_, l_, g_, j_);
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

    float m_;
    float l_;
    float g_;
    float j_;

    float time_step_seconds_;

	//!< Auxiliary functions
    void publish_attitude();
    void publish_angular_rate();
    void controller_output_callback(const ControllerOutputServoTiltAngle::SharedPtr msg);
};

void Simulator::publish_attitude()
{
    ControllerInputAttitude msg{};
    msg.stamp = this->get_clock()->now();
    msg.roll_radians = roll_delta_theta_;
    msg.pitch_radians = pitch_delta_theta_;
    attitude_publisher_->publish(msg);
}

void Simulator::publish_angular_rate()
{
    ControllerInputAngularRate msg{};
    msg.stamp = this->get_clock()->now();
    msg.x_roll_angular_rate_radians_per_second = roll_delta_omega_;
    msg.y_pitch_angular_rate_radians_per_second = pitch_delta_omega_;
    angular_rate_publisher_->publish(msg);
}

void Simulator::controller_output_callback(const ControllerOutputServoTiltAngle::SharedPtr msg)
{
    float roll_delta_gamma = msg->inner_servo_tilt_angle_radians;
    float pitch_delta_gamma = msg->outer_servo_tilt_angle_radians;
    float step = time_step_seconds_;

    //*********//
    //* roll  *//
    //*********//
    {
        float a = roll_delta_gamma * m_ * l_ * g_ / j_;
        roll_delta_omega_ += a * step;
        roll_delta_theta_ += roll_delta_omega_ * step;
    }

    //*********//
    //* pitch *//
    //*********//
    {
        float a = pitch_delta_gamma * m_ * l_ * g_ / j_;
        pitch_delta_omega_ += a * step;
        pitch_delta_theta_ += pitch_delta_omega_ * step;
    }

    // Publish the new state variables
    publish_attitude();
    publish_angular_rate();
}

int main(int argc, char *argv[])
{
	std::cout << "Starting simulator..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Simulator>());

	rclcpp::shutdown();
	return 0;
}
