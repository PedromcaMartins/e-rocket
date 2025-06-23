#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_input_setpoint.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::flight_mode;

/**
 * @brief Node that runs the controller for a 1-degree-of-freedom system
 */
class Controller : public rclcpp::Node
{
public:
	explicit Controller() : Node("controller"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
    controller_timer_{this->create_wall_timer(
        std::chrono::duration<float>(CONTROLLER_DT_SECONDS), 
        std::bind(&Controller::controller_callback, this)
    )},
    attitude_subscriber_{this->create_subscription<ControllerInputAttitude>(
        CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_,
        [this](const ControllerInputAttitude::SharedPtr msg) {
            angle_radians_.store(msg->attitude_radians);
        }
    )},
    angular_rate_subscriber_{this->create_subscription<ControllerInputAngularRate>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const ControllerInputAngularRate::SharedPtr msg) {
            angular_velocity_radians_per_second_.store(msg->angular_rate_radians_per_second);
        }
    )},
    setpoint_subscriber_{this->create_subscription<ControllerInputSetpoint>(
        CONTROLLER_INPUT_SETPOINT_TOPIC, qos_,
        [this](const ControllerInputSetpoint::SharedPtr msg) {
            angle_setpoint_radians_.store(msg->setpoint_radians);
        }
    )},
    servo_tilt_angle_publisher_{this->create_publisher<ControllerOutputServoTiltAngle>(
        CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_
    )},
    motor_thrust_publisher_{this->create_publisher<ControllerOutputMotorThrust>(
        CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_
    )},
    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        std::bind(&Controller::response_flight_mode_callback, this, std::placeholders::_1)
    )}
    {
        this->declare_parameter<float>(CONTROLLER_K_P_PARAM);
        this->declare_parameter<float>(CONTROLLER_K_D_PARAM);
        this->declare_parameter<float>(CONTROLLER_K_I_PARAM);

        k_p_ = this->get_parameter(CONTROLLER_K_P_PARAM).as_double();
        k_d_ = this->get_parameter(CONTROLLER_K_D_PARAM).as_double();
        k_i_ = this->get_parameter(CONTROLLER_K_I_PARAM).as_double();

        // Safety check
        if (k_p_ == NAN || k_d_ == NAN || k_i_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read PID position controller gains correctly.");
            throw std::runtime_error("Gains vector was empty");
        }

        // Print values
        RCLCPP_INFO(this->get_logger(), "gains k_p: %f", k_p_);
        RCLCPP_INFO(this->get_logger(), "gains k_d: %f", k_d_);
        RCLCPP_INFO(this->get_logger(), "gains k_i: %f", k_i_);
	}

private:
    rmw_qos_profile_t qos_profile_;
    rclcpp::QoS qos_;

    //!< Time variables
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Publishers and Subscribers
	rclcpp::Subscription<ControllerInputAttitude>::SharedPtr     attitude_subscriber_;
	rclcpp::Subscription<ControllerInputAngularRate>::SharedPtr  angular_rate_subscriber_;
	rclcpp::Subscription<ControllerInputSetpoint>::SharedPtr     setpoint_subscriber_;
	rclcpp::Publisher<ControllerOutputServoTiltAngle>::SharedPtr servo_tilt_angle_publisher_;
    rclcpp::Publisher<ControllerOutputMotorThrust>::SharedPtr    motor_thrust_publisher_;

    // control algorithm variables
	std::atomic<float> angle_radians_ = 0.0f;
	std::atomic<float> angular_velocity_radians_per_second_ = 0.0f;
	std::atomic<float> angle_setpoint_radians_ = 0.0f;

    // //!< Control algorithm parameters
    float k_p_;
    float k_d_;
    float k_i_;

	//!< Auxiliary functions
    void controller_callback();
    float controller(float angle_radians, float angular_velocity_radians_per_second, float angle_setpoint_radians, float dt_seconds);
	void publish_servo_tilt_angle(float servo_tilt_angle_radians);
	void publish_motor_thrust(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage);

    std::atomic<uint8_t> flight_mode_;
    rclcpp::Subscription<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_get_subscriber_;
	void response_flight_mode_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response);
};

void Controller::response_flight_mode_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response)
{
	flight_mode_.store(response->flight_mode);
}

/**
 * @brief Callback function for the controller
 */
void Controller::controller_callback()
{
    // only run controller when in mission
    if (flight_mode_.load() == FlightMode::IN_MISSION) {
        // Read the sensor data and setpoint
        float delta_theta = angle_radians_.load();
        float delta_omega = angular_velocity_radians_per_second_.load();
        float delta_theta_desired = angle_setpoint_radians_.load();

        // Call the controller function
        float delta_gamma = controller(delta_theta, delta_omega, delta_theta_desired, CONTROLLER_DT_SECONDS);

        // Publish the tilt angle output
        publish_servo_tilt_angle(delta_gamma);

        // Publish the motor thrust
        publish_motor_thrust(CONTROLLER_MOTOR_THRUST_PERCENTAGE, CONTROLLER_MOTOR_THRUST_PERCENTAGE);
    }
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
    float dot_product = delta_theta * k_p_ + delta_omega * k_d_;
    float delta_gamma = -dot_product + zeta_theta * k_i_;

    return delta_gamma;
}

void Controller::publish_servo_tilt_angle(float servo_tilt_angle_radians)
{
    ControllerOutputServoTiltAngle msg{};
    msg.stamp = this->get_clock()->now();
    msg.servo_tilt_angle_radians = servo_tilt_angle_radians;
    servo_tilt_angle_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void Controller::publish_motor_thrust(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage)
{
	ControllerOutputMotorThrust msg{};
	msg.stamp = this->get_clock()->now();
	msg.upwards_motor_thrust_percentage = upwards_motor_thrust_percentage;
	msg.downwards_motor_thrust_percentage = downwards_motor_thrust_percentage;
	motor_thrust_publisher_->publish(msg);
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
