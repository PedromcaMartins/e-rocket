#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <one_degree_freedom/msg/controller_debug.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>

#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants;
using namespace one_degree_freedom::frame_transforms;

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

        controller_debug_subscriber_ = this->create_subscription<ControllerDebug>(
            CONTROLLER_DEBUG_TOPIC, qos,
            std::bind(&Simulator::controller_output_callback, this, std::placeholders::_1)
        );

        attitude_publisher_ = this->create_publisher<VehicleAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos
        );

        angular_rate_publisher_ = this->create_publisher<VehicleAngularVelocity>(
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
	rclcpp::Publisher<VehicleAttitude>::SharedPtr        attitude_publisher_;
	rclcpp::Publisher<VehicleAngularVelocity>::SharedPtr angular_rate_publisher_;
	rclcpp::Subscription<ControllerDebug>::SharedPtr     controller_debug_subscriber_;

    //!< State variables - to be updated by the simulator
    float roll_angle_ = 0.0f;         // pitch angle
    float roll_angular_rate_ = 0.0f;  // angular position

    float pitch_angle_ = 0.0f;        // pitch angle
    float pitch_angular_rate_ = 0.0f; // angular position

    float yaw_angle_ = 0.0f;          // pitch angle
    float yaw_angular_rate_ = 0.0f;   // angular position

    float m_;
    float l_;
    float g_;
    float j_;

    float time_step_seconds_;

	//!< Auxiliary functions
    void publish_attitude();
    void publish_angular_rate();
    void controller_output_callback(const ControllerDebug::SharedPtr msg);
};

void Simulator::publish_attitude()
{
    VehicleAttitude msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    Eigen::Quaterniond q = euler_radians_to_quaternion(
        EulerAngle { roll_angle_, pitch_angle_, yaw_angle_ }
    );

    msg.q[0] = q.w();
    msg.q[1] = q.x();
    msg.q[2] = q.y();
    msg.q[3] = q.z();
    attitude_publisher_->publish(msg);
}

void Simulator::publish_angular_rate()
{
    VehicleAngularVelocity msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.xyz[0] = roll_angular_rate_;
    msg.xyz[1] = pitch_angular_rate_;
    msg.xyz[2] = yaw_angular_rate_;
    angular_rate_publisher_->publish(msg);
}

void Simulator::controller_output_callback(const ControllerDebug::SharedPtr msg)
{
    float roll_inner_servo_tilt_angle = msg->roll_inner_servo_tilt_angle;
    float pitch_outer_servo_tilt_angle = msg->pitch_outer_servo_tilt_angle;
    float yaw_delta_motor_pwm = msg->yaw_delta_motor_pwm;
    float step = time_step_seconds_;

    //*********//
    //* roll  *//
    //*********//
    {
        float a = roll_inner_servo_tilt_angle * m_ * l_ * g_ / j_;
        roll_angular_rate_ += a * step;
        roll_angle_ += roll_angular_rate_ * step;
    }

    //*********//
    //* pitch *//
    //*********//
    {
        float a = pitch_outer_servo_tilt_angle * m_ * l_ * g_ / j_;
        pitch_angular_rate_ += a * step;
        pitch_angle_ += pitch_angular_rate_ * step;
    }

    //*********//
    //*  yaw  *//
    //*********//
    {
        float a = yaw_delta_motor_pwm * m_ * l_ * g_ / j_;
        yaw_angular_rate_ += a * step;
        yaw_angle_ += yaw_angular_rate_ * step;
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
