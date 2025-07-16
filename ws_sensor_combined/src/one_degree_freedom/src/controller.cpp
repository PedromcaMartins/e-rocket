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

class PIDController
{
public:
    PIDController(double k_p, double k_d, double k_i, double dt)
        : k_p_(k_p), k_d_(k_d), k_i_(k_i),
            integrated_error_(0.0f), dt_(dt) 
            {
                // Safety check
                if (k_p == NAN || k_d == NAN || k_i == NAN) {
                    RCLCPP_ERROR(rclcpp::get_logger("PIDController"), "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                // Safety check
                if (dt <= 0.0f || dt == NAN) {
                    RCLCPP_ERROR(rclcpp::get_logger("PIDController"), "Invalid time step provided.");
                    throw std::runtime_error("Time step invalid");
                }
            }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @param theta Current measurement
        * @param omega Current measurement derivative
        * @param theta_desired Desired setpoint
        * @return The computed control input
    */
    double compute(double theta, double omega, double theta_desired) {
        // Update the integrated error
        integrated_error_ = integrated_error_ + (theta_desired - theta) * dt_; 

        // Compute control input
        double dot_product = theta * k_p_ + omega * k_d_;
        double gamma = -dot_product + integrated_error_ * k_i_;
        return gamma;
    }

private:
    const double k_p_;
    const double k_d_;
    const double k_i_;
    double integrated_error_;
    const double dt_;
};

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
            yaw_radians_.store(msg->yaw_radians);
        }
    )},
    angular_rate_subscriber_{this->create_subscription<ControllerInputAngularRate>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const ControllerInputAngularRate::SharedPtr msg) {
            x_roll_angular_rate_radians_per_second_.store(msg->x_roll_angular_rate_radians_per_second);
            y_pitch_angular_rate_radians_per_second_.store(msg->y_pitch_angular_rate_radians_per_second);
            z_yaw_angular_rate_radians_per_second_.store(msg->z_yaw_angular_rate_radians_per_second);
        }
    )},
    setpoint_subscriber_{this->create_subscription<ControllerInputSetpoint>(
        CONTROLLER_INPUT_SETPOINT_TOPIC, qos_,
        [this](const ControllerInputSetpoint::SharedPtr msg) {
            roll_setpoint_radians_.store(msg->roll_setpoint_radians);
            pitch_setpoint_radians_.store(msg->pitch_setpoint_radians);
            yaw_setpoint_radians_.store(msg->yaw_setpoint_radians);
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

    yaw_radians_{0.0f},
    z_yaw_angular_rate_radians_per_second_{0.0f},
    yaw_setpoint_radians_{0.0f},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        std::bind(&ControllerInnerLoop::response_flight_mode_callback, this, std::placeholders::_1)
    )}
    {
        this->declare_parameter<double>(CONTROLLER_FREQUENCY_HERTZ_PARAM);
        double controllers_freq = this->get_parameter(CONTROLLER_FREQUENCY_HERTZ_PARAM).as_double();
        RCLCPP_INFO(this->get_logger(), "Controllers freq: %f", controllers_freq);
        double controllers_dt = 1.0 / controllers_freq;


        this->declare_parameter<double>(CONTROLLER_MOTOR_THRUST_PERCENTAGE_PARAM);
        motor_thrust_percentage_ = this->get_parameter(CONTROLLER_MOTOR_THRUST_PERCENTAGE_PARAM).as_double();
        if (motor_thrust_percentage_ < 0.0f || motor_thrust_percentage_ > 1.0f || motor_thrust_percentage_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read motor thrust correctly.");
            throw std::runtime_error("Motor thrust invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Motor thrust percentage: %f", motor_thrust_percentage_);


        this->declare_parameter<bool>(CONTROLLER_ROLL_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_ROLL_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Roll is active");
            this->declare_parameter<double>(CONTROLLER_ROLL_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_ROLL_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_ROLL_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_ROLL_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_ROLL_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_ROLL_K_I_PARAM).as_double();

            roll_controller_.emplace(PIDController(k_p, k_d, k_i, controllers_dt));

            RCLCPP_INFO(this->get_logger(), "gains k_p: %f", k_p);
            RCLCPP_INFO(this->get_logger(), "gains k_d: %f", k_d);
            RCLCPP_INFO(this->get_logger(), "gains k_i: %f", k_i);
        } else {
            RCLCPP_INFO(this->get_logger(), "Roll is not active");
            roll_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_PITCH_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_PITCH_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Pitch is active");
            this->declare_parameter<double>(CONTROLLER_PITCH_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_PITCH_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_PITCH_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_PITCH_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_PITCH_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_PITCH_K_I_PARAM).as_double();

            pitch_controller_.emplace(PIDController(k_p, k_d, k_i, controllers_dt));

            RCLCPP_INFO(this->get_logger(), "gains k_p: %f", k_p);
            RCLCPP_INFO(this->get_logger(), "gains k_d: %f", k_d);
            RCLCPP_INFO(this->get_logger(), "gains k_i: %f", k_i);
        } else {
            RCLCPP_INFO(this->get_logger(), "Pitch is not active");
            pitch_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_YAW_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_YAW_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Yaw is active");
            this->declare_parameter<double>(CONTROLLER_YAW_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_YAW_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_YAW_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_YAW_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_YAW_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_YAW_K_I_PARAM).as_double();

            yaw_controller_.emplace(PIDController(k_p, k_d, k_i, controllers_dt));

            RCLCPP_INFO(this->get_logger(), "gains k_p: %f", k_p);
            RCLCPP_INFO(this->get_logger(), "gains k_d: %f", k_d);
            RCLCPP_INFO(this->get_logger(), "gains k_i: %f", k_i);
        } else {
            RCLCPP_INFO(this->get_logger(), "Yaw is not active");
            yaw_controller_ = std::nullopt;
        }


        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<float>(controllers_dt), 
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
    rclcpp::Publisher<ControllerDebug>::SharedPtr                controller_debug_publisher_;

    // control algorithm variables
	std::atomic<float> roll_radians_;
	std::atomic<float> x_roll_angular_rate_radians_per_second_;
	std::atomic<float> roll_setpoint_radians_;
    std::optional<PIDController> roll_controller_;

	std::atomic<float> pitch_radians_;
	std::atomic<float> y_pitch_angular_rate_radians_per_second_;
	std::atomic<float> pitch_setpoint_radians_;
    std::optional<PIDController> pitch_controller_;

	std::atomic<float> yaw_radians_;
	std::atomic<float> z_yaw_angular_rate_radians_per_second_;
	std::atomic<float> yaw_setpoint_radians_;
    std::optional<PIDController> yaw_controller_;

    float motor_thrust_percentage_;

	//!< Auxiliary functions
    void controller_callback();
    void publish_controller_debug(
        const float roll_angle, 
        const float roll_angular_velocity, 
        const float roll_angle_setpoint, 
        const float roll_tilt_angle,

        const float pitch_angle, 
        const float pitch_angular_velocity, 
        const float pitch_angle_setpoint, 
        const float pitch_tilt_angle,

        const float yaw_radians, 
        const float yaw_angular_velocity, 
        const float yaw_setpoint_radians, 
        const float differential_motor_thrust_percentage
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

float limit_range_motor_thrust(float motor_pwm) {
    if (motor_pwm > 1.0f) {
        motor_pwm = 1.0f;
    } else if (motor_pwm < 0.0f) {
        motor_pwm = 0.0f;
    }

    return motor_pwm;
}

/**
 * @brief Callback function for the controller
 */
void ControllerInnerLoop::controller_callback()
{
    // only run controller when in mission
    if (flight_mode_.load() == FlightMode::IN_MISSION) {
        float outer_servo_pwm = 0.0f;
        float inner_servo_pwm = 0.0f;
        float differential_motor_thrust_percentage = 0.0f;
        float upwards_motor_thrust_percentage = motor_thrust_percentage_;
        float downwards_motor_thrust_percentage = motor_thrust_percentage_;

        float roll_angle = roll_radians_.load();
        float roll_angular_velocity = x_roll_angular_rate_radians_per_second_.load();
        float roll_angle_setpoint = roll_setpoint_radians_.load();
        if (roll_controller_.has_value()) {
            // Compute the control input for roll
            inner_servo_pwm = roll_controller_->compute(
                roll_angle, 
                roll_angular_velocity, 
                roll_angle_setpoint
            );
        }

        float pitch_angle = pitch_radians_.load();
        float pitch_angular_velocity = y_pitch_angular_rate_radians_per_second_.load();
        float pitch_angle_setpoint = pitch_setpoint_radians_.load();
        if (pitch_controller_.has_value()) {
            // Compute the control input for pitch
            outer_servo_pwm = pitch_controller_->compute(
                pitch_angle, 
                pitch_angular_velocity, 
                pitch_angle_setpoint
            );
        }

        float yaw_angle = yaw_radians_.load();
        float yaw_angular_velocity = z_yaw_angular_rate_radians_per_second_.load();
        float yaw_angle_setpoint = yaw_setpoint_radians_.load();
        if (yaw_controller_.has_value()) {
            // Compute the control input for yaw
            differential_motor_thrust_percentage = yaw_controller_->compute(
                yaw_angle, 
                yaw_angular_velocity, 
                yaw_angle_setpoint
            );
        }

        // Publish the tilt angle output
        publish_servo_tilt_angle(
            limit_range_servo_tilt(outer_servo_pwm), 
            limit_range_servo_tilt(inner_servo_pwm)
        );

        // Publish the motor thrust
        publish_motor_thrust(
            limit_range_motor_thrust(upwards_motor_thrust_percentage), 
            limit_range_motor_thrust(downwards_motor_thrust_percentage)
        );


        // Publish the controller debug information
        publish_controller_debug(
            roll_angle, 
            roll_angular_velocity, 
            roll_angle_setpoint, 
            inner_servo_pwm,

            pitch_angle,
            pitch_angular_velocity,
            pitch_angle_setpoint,
            outer_servo_pwm,

            yaw_angle,
            yaw_angular_velocity,
            yaw_angle_setpoint,
            differential_motor_thrust_percentage
        );
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
    const float outer_servo_tilt_angle,

    const float yaw_angle, 
    const float yaw_angular_velocity, 
    const float yaw_angle_setpoint, 
    const float differential_motor_thrust_percentage
) {
    auto to_degrees = [](float radians) {
        return radians * (180.0f / M_PI);
    };

    float roll_angle_degrees = to_degrees(roll_angle);
    float roll_angular_velocity_degrees_per_second = to_degrees(roll_angular_velocity);
    float roll_angle_setpoint_degrees = to_degrees(roll_angle_setpoint);
    float inner_servo_tilt_angle_degrees = to_degrees(inner_servo_tilt_angle);

    float pitch_angle_degrees = to_degrees(pitch_angle);
    float pitch_angular_velocity_degrees_per_second = to_degrees(pitch_angular_velocity);
    float pitch_angle_setpoint_degrees = to_degrees(pitch_angle_setpoint);
    float outer_servo_tilt_angle_degrees = to_degrees(outer_servo_tilt_angle);

    float yaw_angle_degrees = to_degrees(yaw_angle);
    float yaw_angular_velocity_degrees_per_second = to_degrees(yaw_angular_velocity);
    float yaw_angle_setpoint_degrees = to_degrees(yaw_angle_setpoint);


    RCLCPP_INFO(this->get_logger(), "Roll  angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", roll_angle_degrees, roll_angular_velocity_degrees_per_second, roll_angle_setpoint_degrees, inner_servo_tilt_angle_degrees);
    RCLCPP_INFO(this->get_logger(), "Pitch angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", pitch_angle_degrees, pitch_angular_velocity_degrees_per_second, pitch_angle_setpoint_degrees, outer_servo_tilt_angle_degrees);
    RCLCPP_INFO(this->get_logger(), "Yaw   angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", yaw_angle_degrees, yaw_angular_velocity_degrees_per_second, yaw_angle_setpoint_degrees, differential_motor_thrust_percentage);


    ControllerDebug msg{};
    msg.roll_angle = roll_angle_degrees;
    msg.roll_angular_velocity = roll_angular_velocity_degrees_per_second;
    msg.roll_angle_setpoint = roll_angle_setpoint_degrees;
    msg.inner_servo_tilt_angle = inner_servo_tilt_angle_degrees;

    msg.pitch_angle = pitch_angle_degrees;
    msg.pitch_angular_velocity = pitch_angular_velocity_degrees_per_second;
    msg.pitch_angle_setpoint = pitch_angle_setpoint_degrees;
    msg.outer_servo_tilt_angle = outer_servo_tilt_angle_degrees;

    msg.yaw_angle = yaw_angle_degrees;
    msg.yaw_angular_velocity = yaw_angular_velocity_degrees_per_second;
    msg.yaw_angle_setpoint = yaw_angle_setpoint_degrees;
    msg.reference_motor_thrust_percentage = differential_motor_thrust_percentage;

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
