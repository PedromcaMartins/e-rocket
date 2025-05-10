#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_input_setpoint.hpp>
#include <one_degree_freedom/msg/controller_output_tilt_angle.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants;

/**
 * @brief Node that runs the controller for a 1-degree-of-freedom system
 */
class Controller : public rclcpp::Node
{
public:
	explicit Controller() : Node("controller")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<float>(CONTROLLER_DT), 
            std::bind(&Controller::controller_callback, this)
        );

        attitude_subscriber_ = this->create_subscription<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE, qos,
            [this](const ControllerInputAttitude::SharedPtr msg) {
                angle_ = msg->attitude;
            }
        );

        angular_rate_subscriber_ = this->create_subscription<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE, qos,
            [this](const ControllerInputAngularRate::SharedPtr msg) {
                angular_velocity_ = msg->angular_rate;
            }
        );

        setpoint_subscriber_ = this->create_subscription<ControllerInputSetpoint>(
            CONTROLLER_INPUT_SETPOINT, qos,
            [this](const ControllerInputSetpoint::SharedPtr msg) {
                angle_setpoint_ = msg->setpoint;
            }
        );

        tilt_angle_publisher_ = this->create_publisher<ControllerOutputTiltAngle>(
            CONTROLLER_OUTPUT_TILT_ANGLE, qos
        );
	}

private:
    //!< Time variables
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Publishers and Subscribers
	rclcpp::Subscription<ControllerInputAttitude>::SharedPtr    attitude_subscriber_;
	rclcpp::Subscription<ControllerInputAngularRate>::SharedPtr angular_rate_subscriber_;
	rclcpp::Subscription<ControllerInputSetpoint>::SharedPtr    setpoint_subscriber_;
	rclcpp::Publisher<ControllerOutputTiltAngle>::SharedPtr     tilt_angle_publisher_;

    // control algorithm variables
	std::atomic<float> angle_ = 0.0f;
	std::atomic<float> angular_velocity_ = 0.0f;
	std::atomic<float> angle_setpoint_ = 0.0f;

    //!< Control algorithm parameters
    const float k_p = 0.3350f; // Proportional gain
    const float k_d = 0.1616f; // Derivative gain
    const float k_i = 0.3162f; // Integral gain

	//!< Auxiliary functions
    void controller_callback();
    float controller(float angle, float angular_velocity, float angle_setpoint, float dt);
	void publish_tilt_angle(float tilt_angle);
};

/**
 * @brief Callback function for the controller
 */
void Controller::controller_callback()
{
    // Get the time step
    // Read the sensor data and setpoint
    float delta_theta = angle_.load();
    float delta_omega = angular_velocity_.load();
    float delta_theta_desired = angle_setpoint_.load();

    // Call the controller function
    float delta_gamma = controller(delta_theta, delta_omega, delta_theta_desired, CONTROLLER_DT);

    // Publish the tilt angle output
    publish_tilt_angle(delta_gamma);
}

/**
 * @brief Controller function
 * @param delta_theta Current angle (in radians)
 * @param delta_omega Current angular velocity (in radians per second)
 * @param delta_theta_desired Desired angle setpoint (in radians)
 * @param dt Time step for the controller (in seconds)
 * @return Tilt angle for the servo
 */
float Controller::controller(float delta_theta, float delta_omega, float delta_theta_desired, float dt)
{
    // Update the integrated error
    static float zeta_theta = 0.0f;
    zeta_theta = zeta_theta + (delta_theta_desired - delta_theta) * dt; 

    // Compute control input
    float dot_product = delta_theta * k_p + delta_omega * k_d;
    float delta_gamma = -dot_product + zeta_theta * k_i;

    return delta_gamma;
}

void Controller::publish_tilt_angle(float tilt_angle)
{
    ControllerOutputTiltAngle msg{};
    msg.stamp = this->get_clock()->now();
    msg.tilt_angle = tilt_angle;
    tilt_angle_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Controller>());

	rclcpp::shutdown();
	return 0;
}
