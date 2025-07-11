#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>
#include <one_degree_freedom/msg/controller_input_setpoint.hpp>
#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>
#include <one_degree_freedom/msg/controller_debug.hpp>
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
class ControllerInnerLoop : public rclcpp::Node
{
public:
	explicit ControllerInnerLoop() : Node("controller_inner_loop"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

    attitude_subscriber_{this->create_subscription<ControllerInputAttitude>(
        CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_,
        [this](const ControllerInputAttitude::SharedPtr msg) {
            roll_radians_.store(msg->roll_radians);
            pitch_radians_.store(msg->pitch_radians);
        }
    )},
    angular_rate_subscriber_{this->create_subscription<ControllerInputAngularRate>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const ControllerInputAngularRate::SharedPtr msg) {
            x_roll_angular_rate_radians_per_second_.store(msg->x_roll_angular_rate_radians_per_second);
            y_pitch_angular_rate_radians_per_second_.store(msg->y_pitch_angular_rate_radians_per_second);
        }
    )},
    setpoint_subscriber_{this->create_subscription<ControllerInputSetpoint>(
        CONTROLLER_INPUT_SETPOINT_TOPIC, qos_,
        [this](const ControllerInputSetpoint::SharedPtr msg) {
            roll_setpoint_radians_.store(msg->roll_setpoint_radians);
            pitch_setpoint_radians_.store(msg->pitch_setpoint_radians);
        }
    )},
    servo_tilt_angle_publisher_{this->create_publisher<ControllerOutputServoTiltAngle>(
        CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_
    )},
    motor_thrust_publisher_{this->create_publisher<ControllerOutputMotorThrust>(
        CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_
    )},
    controller_debug_publisher_{this->create_publisher<ControllerDebug>(
        CONTROLLER_DEBUG_TOPIC, qos_
    )},

    roll_radians_{0.0f},
    x_roll_angular_rate_radians_per_second_{0.0f},
    roll_setpoint_radians_{0.0f},
    pitch_radians_{0.0f},
    y_pitch_angular_rate_radians_per_second_{0.0f},
    pitch_setpoint_radians_{0.0f},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        std::bind(&ControllerInnerLoop::response_flight_mode_callback, this, std::placeholders::_1)
    )}
    {
        this->declare_parameter<bool>(CONTROLLER_ROLL_ACTIVE_PARAM);
        this->declare_parameter<bool>(CONTROLLER_PITCH_ACTIVE_PARAM);

        roll_active_ = this->get_parameter(CONTROLLER_ROLL_ACTIVE_PARAM).as_bool();
        pitch_active_ = this->get_parameter(CONTROLLER_PITCH_ACTIVE_PARAM).as_bool();

        RCLCPP_INFO(this->get_logger(), "Roll active: %s", roll_active_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Pitch active: %s", pitch_active_ ? "true" : "false");


        this->declare_parameter<float>(CONTROLLER_ROLL_K_P_PARAM);
        this->declare_parameter<float>(CONTROLLER_ROLL_K_D_PARAM);
        this->declare_parameter<float>(CONTROLLER_ROLL_K_I_PARAM);
        this->declare_parameter<float>(CONTROLLER_PITCH_K_P_PARAM);
        this->declare_parameter<float>(CONTROLLER_PITCH_K_D_PARAM);
        this->declare_parameter<float>(CONTROLLER_PITCH_K_I_PARAM);

        roll_k_p_ = this->get_parameter(CONTROLLER_ROLL_K_P_PARAM).as_double();
        roll_k_d_ = this->get_parameter(CONTROLLER_ROLL_K_D_PARAM).as_double();
        roll_k_i_ = this->get_parameter(CONTROLLER_ROLL_K_I_PARAM).as_double();
        pitch_k_p_ = this->get_parameter(CONTROLLER_PITCH_K_P_PARAM).as_double();
        pitch_k_d_ = this->get_parameter(CONTROLLER_PITCH_K_D_PARAM).as_double();
        pitch_k_i_ = this->get_parameter(CONTROLLER_PITCH_K_I_PARAM).as_double();

        // Safety check
        if (roll_k_p_ == NAN || roll_k_d_ == NAN || roll_k_i_ == NAN || pitch_k_p_ == NAN || pitch_k_d_ == NAN || pitch_k_i_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read PID position controller inner loop gains correctly.");
            throw std::runtime_error("Gains vector invalid");
        }

        // Print values
        RCLCPP_INFO(this->get_logger(), "gains roll k_p: %f", roll_k_p_);
        RCLCPP_INFO(this->get_logger(), "gains roll k_d: %f", roll_k_d_);
        RCLCPP_INFO(this->get_logger(), "gains roll k_i: %f", roll_k_i_);
        RCLCPP_INFO(this->get_logger(), "gains pitch k_p: %f", pitch_k_p_);
        RCLCPP_INFO(this->get_logger(), "gains pitch k_d: %f", pitch_k_d_);
        RCLCPP_INFO(this->get_logger(), "gains pitch k_i: %f", pitch_k_i_);


        this->declare_parameter<float>(CONTROLLER_PERIOD_SECONDS_PARAM);
        this->declare_parameter<float>(CONTROLLER_MOTOR_THRUST_PERCENTAGE_PARAM);

        time_step_seconds_ = this->get_parameter(CONTROLLER_PERIOD_SECONDS_PARAM).as_double();
        motor_thrust_percentage_ = this->get_parameter(CONTROLLER_MOTOR_THRUST_PERCENTAGE_PARAM).as_double();

        // Safety check
        if (time_step_seconds_ <= 0.0f || time_step_seconds_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read controller time step correctly.");
            throw std::runtime_error("Time step invalid");
        }
        if (motor_thrust_percentage_ < 0.0f || motor_thrust_percentage_ > 1.0f || motor_thrust_percentage_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read motor thrust correctly.");
            throw std::runtime_error("Motor thrust invalid");
        }

        RCLCPP_INFO(this->get_logger(), "Controller period: %f seconds", time_step_seconds_);
        RCLCPP_INFO(this->get_logger(), "Motor thrust percentage: %f", motor_thrust_percentage_);


        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<float>(time_step_seconds_), 
            std::bind(&ControllerInnerLoop::controller_callback, this)
        );
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
    rclcpp::Publisher<ControllerDebug>::SharedPtr controller_debug_publisher_;

    // control algorithm variables
	std::atomic<float> roll_radians_;
	std::atomic<float> x_roll_angular_rate_radians_per_second_;
	std::atomic<float> roll_setpoint_radians_;

	std::atomic<float> pitch_radians_;
	std::atomic<float> y_pitch_angular_rate_radians_per_second_;
	std::atomic<float> pitch_setpoint_radians_;

    bool roll_active_;
    bool pitch_active_;

    // //!< Control algorithm parameters
    float roll_k_p_;
    float roll_k_d_;
    float roll_k_i_;
    float pitch_k_p_;
    float pitch_k_d_;
    float pitch_k_i_;

    float time_step_seconds_;
    float motor_thrust_percentage_;

	//!< Auxiliary functions
    void controller_callback();
    void controller(
        const float roll_radians, 
        const float x_roll_angular_rate_radians_per_second, 
        const float roll_setpoint_radians, 
        float* inner_servo_tilt_angle,
        const float pitch_radians, 
        const float y_pitch_angular_rate_radians_per_second, 
        const float pitch_setpoint_radians, 
        float* outer_servo_tilt_angle,
        const float dt_seconds
    );
    void publish_controller_debug(
        const float roll_angle, 
        const float roll_angular_velocity, 
        const float roll_angle_setpoint, 
        const float roll_tilt_angle,
        const float pitch_angle, 
        const float pitch_angular_velocity, 
        const float pitch_angle_setpoint, 
        const float pitch_tilt_angle
    );

    void publish_servo_tilt_angle(const float outer_servo_tilt_angle_radians, const float inner_servo_tilt_angle_radians);
	void publish_motor_thrust(const float upwards_motor_thrust_percentage, const float downwards_motor_thrust_percentage);

    std::atomic<uint8_t> flight_mode_;
    rclcpp::Subscription<FlightMode>::SharedPtr flight_mode_get_subscriber_;
	void response_flight_mode_callback(const std::shared_ptr<FlightMode> response);
};

void ControllerInnerLoop::response_flight_mode_callback(const std::shared_ptr<FlightMode> response)
{
	flight_mode_.store(response->flight_mode);
}

float limit_range_servo_tilt(float servo_pwm) {
    if (servo_pwm > 1.0f) {
        servo_pwm = 1.0f;
    } else if (servo_pwm < -1.0f) {
        servo_pwm = -1.0f;
    }

    return servo_pwm;
}

/**
 * @brief Callback function for the controller
 */
void ControllerInnerLoop::controller_callback()
{
    // only run controller when in mission
    if (flight_mode_.load() == FlightMode::IN_MISSION) {
        // Read the sensor data and setpoint
        float roll_delta_theta = roll_radians_.load();
        float roll_delta_omega = x_roll_angular_rate_radians_per_second_.load();
        float roll_delta_theta_desired = roll_setpoint_radians_.load();
        float roll_delta_gamma = 0.0f;

        float pitch_delta_theta = pitch_radians_.load();
        float pitch_delta_omega = y_pitch_angular_rate_radians_per_second_.load();
        float pitch_delta_theta_desired = pitch_setpoint_radians_.load();
        float pitch_delta_gamma = 0.0f;

        // Call the controller function
        controller(
            roll_delta_theta, 
            roll_delta_omega, 
            roll_delta_theta_desired, 
            &roll_delta_gamma,
            pitch_delta_theta, 
            pitch_delta_omega, 
            pitch_delta_theta_desired,
            &pitch_delta_gamma,
            time_step_seconds_
        );
        publish_controller_debug(
            roll_delta_theta, 
            roll_delta_omega, 
            roll_delta_theta_desired, 
            roll_delta_gamma,
            pitch_delta_theta, 
            pitch_delta_omega, 
            pitch_delta_theta_desired,
            pitch_delta_gamma
        );

        // Publish the tilt angle output
        float inner_servo_pwm = limit_range_servo_tilt(roll_delta_gamma);
        float outer_servo_pwm = limit_range_servo_tilt(pitch_delta_gamma);
        publish_servo_tilt_angle(outer_servo_pwm, inner_servo_pwm);

        // Publish the motor thrust
        publish_motor_thrust(motor_thrust_percentage_, motor_thrust_percentage_);
    }
}

/**
 * @brief ControllerInnerLoop function
 * @param delta_theta Current angle (in radians)
 * @param delta_omega Current angular velocity (in radians per second)
 * @param delta_theta_desired Desired angle setpoint (in radians)
 * @param dt Time step for the controller (in seconds)
 * @return Tilt angle for the servo
 */
void ControllerInnerLoop::controller(
    const float roll_delta_theta, 
    const float roll_delta_omega, 
    const float roll_delta_theta_desired, 
    float* roll_delta_gamma,
    const float pitch_delta_theta, 
    const float pitch_delta_omega, 
    const float pitch_delta_theta_desired, 
    float* pitch_delta_gamma,
    const float dt
)
{
    //*********//
    //* roll  *//
    //*********//
    if (roll_active_) {
        // Update the integrated error
        static float zeta_theta = 0.0f;
        zeta_theta = zeta_theta + (roll_delta_theta_desired - roll_delta_theta) * dt; 

        // Compute control input
        float dot_product = roll_delta_theta * roll_k_p_ + roll_delta_omega * roll_k_d_;
        *roll_delta_gamma = -dot_product + zeta_theta * roll_k_i_;
    }

    //*********//
    //* pitch *//
    //*********//
    if (pitch_active_) {
        // Update the integrated error
        static float zeta_theta = 0.0f;
        zeta_theta = zeta_theta + (pitch_delta_theta_desired - pitch_delta_theta) * dt; 

        // Compute control input
        float dot_product = pitch_delta_theta * pitch_k_p_ + pitch_delta_omega * pitch_k_d_;
        *pitch_delta_gamma = -dot_product + zeta_theta * pitch_k_i_;
    }
}

void ControllerInnerLoop::publish_controller_debug(
    const float roll_angle, 
    const float roll_angular_velocity, 
    const float roll_angle_setpoint, 
    const float inner_servo_tilt_angle,

    const float pitch_angle, 
    const float pitch_angular_velocity, 
    const float pitch_angle_setpoint, 
    const float outer_servo_tilt_angle
) {
    RCLCPP_INFO(this->get_logger(), "Roll  angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", roll_angle, roll_angular_velocity, roll_angle_setpoint, inner_servo_tilt_angle);
    RCLCPP_INFO(this->get_logger(), "Pitch angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", pitch_angle, pitch_angular_velocity, pitch_angle_setpoint, outer_servo_tilt_angle);

    ControllerDebug msg{};
    msg.roll_angle = roll_angle;
    msg.roll_angular_velocity = roll_angular_velocity;
    msg.roll_angle_setpoint = roll_angle_setpoint;
    msg.pitch_angle = pitch_angle;
    msg.pitch_angular_velocity = pitch_angular_velocity;
    msg.pitch_angle_setpoint = pitch_angle_setpoint;
    msg.inner_servo_tilt_angle = inner_servo_tilt_angle;
    msg.outer_servo_tilt_angle = outer_servo_tilt_angle;
    msg.stamp = this->get_clock()->now();

    controller_debug_publisher_->publish(msg);
}

void ControllerInnerLoop::publish_servo_tilt_angle(const float outer_servo_tilt_angle_radians, const float inner_servo_tilt_angle_radians)
{
    ControllerOutputServoTiltAngle msg{};
    msg.stamp = this->get_clock()->now();
    msg.outer_servo_tilt_angle_radians = outer_servo_tilt_angle_radians;
    msg.inner_servo_tilt_angle_radians = inner_servo_tilt_angle_radians;
    servo_tilt_angle_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void ControllerInnerLoop::publish_motor_thrust(const float upwards_motor_thrust_percentage, const float downwards_motor_thrust_percentage)
{
	ControllerOutputMotorThrust msg{};
	msg.stamp = this->get_clock()->now();
	msg.upwards_motor_thrust_percentage = upwards_motor_thrust_percentage;
	msg.downwards_motor_thrust_percentage = downwards_motor_thrust_percentage;
	motor_thrust_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard controller inner loop node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControllerInnerLoop>());

	rclcpp::shutdown();
	return 0;
}
