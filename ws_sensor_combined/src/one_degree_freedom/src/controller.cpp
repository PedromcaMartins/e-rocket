#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <one_degree_freedom/msg/controller_debug.hpp>
#include <one_degree_freedom/msg/allocator_debug.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants;
using namespace one_degree_freedom::constants::setpoint;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::flight_mode;
using namespace one_degree_freedom::frame_transforms;

class Allocator {
public:
    // The pwm values are limited by their operating range
    struct ServoAllocatorOutput {
        double inner_servo_pwm;
        double outer_servo_pwm;
    };

    struct MotorAllocatorOutput {
        double upwards_motor_pwm;
        double downwards_motor_pwm;
    };

    Allocator(double thrust_curve_m, double thrust_curve_b, double g, 
        double servo_max_tilt_angle_degrees, double motor_max_pwm
    )   : thrust_curve_m_(thrust_curve_m), thrust_curve_b_(thrust_curve_b), g_(g), 
            servo_max_tilt_angle_degrees_(servo_max_tilt_angle_degrees), motor_max_pwm_(motor_max_pwm)
            {
                if (thrust_curve_m == NAN || thrust_curve_b == NAN || g <= 0.0f || g == NAN ||
                    servo_max_tilt_angle_degrees <= 0.0f || servo_max_tilt_angle_degrees > 90.0f ||
                    servo_max_tilt_angle_degrees == NAN || motor_max_pwm < 0.0f || motor_max_pwm > 1.0 || motor_max_pwm == NAN
                ) {
                    RCLCPP_ERROR(rclcpp::get_logger("allocator"), "Invalid parameters for allocator.");
                    throw std::runtime_error("Allocator parameters invalid");
                }
                RCLCPP_INFO(rclcpp::get_logger("allocator"), "Thrust curve: x * %f + %f", thrust_curve_m_, thrust_curve_b_);
                RCLCPP_INFO(rclcpp::get_logger("allocator"), "Gravitational acceleration: %f", g_);
                RCLCPP_INFO(rclcpp::get_logger("allocator"), "Servo max tilt angle degrees: %f", servo_max_tilt_angle_degrees_);
                RCLCPP_INFO(rclcpp::get_logger("allocator"), "Motor max pwm: %f", motor_max_pwm_);
            }

    ServoAllocatorOutput compute_servo_allocation(
        double inner_servo_tilt_angle_radians, 
        double outer_servo_tilt_angle_radians
    ) {
        ServoAllocatorOutput output;

        double inner_servo_pwm = servo_curve_tilt_radians_to_pwm(inner_servo_tilt_angle_radians);
        double outer_servo_pwm = servo_curve_tilt_radians_to_pwm(outer_servo_tilt_angle_radians);

        output.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        output.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);

        return output;
    }

    MotorAllocatorOutput compute_motor_allocation(
        double delta_motor_pwm, 
        double average_motor_thrust_newtons
    ) {
        MotorAllocatorOutput output;

        double average_motor_pwm = motor_thrust_curve_newtons_to_pwm(average_motor_thrust_newtons);
        double upwards_motor_pwm = average_motor_pwm - delta_motor_pwm / 2.0f;
        double downwards_motor_pwm = average_motor_pwm + delta_motor_pwm / 2.0f;

        output.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        output.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

        return output;
    }

    double motor_thrust_curve_pwm_to_newtons(double motor_pwm) {
        return (motor_pwm * thrust_curve_m_ + thrust_curve_b_) / 1000.0f * g_;
    }

    double motor_thrust_curve_newtons_to_pwm(double thrust_newtons) {
        return ((thrust_newtons * 1000.0f) / g_ - thrust_curve_b_) / thrust_curve_m_;
    }

private: 
    double thrust_curve_m_;
    double thrust_curve_b_;
    double g_;

    double servo_max_tilt_angle_degrees_;
    double motor_max_pwm_;

    double limit_range_servo_pwm(double servo_pwm) {
        servo_pwm = (servo_pwm > 1.0f)? 1.0f : servo_pwm;
        servo_pwm = (servo_pwm < -1.0f)? -1.0f : servo_pwm;

        return servo_pwm;
    }

    double limit_range_motor_pwm(double motor_pwm) {
        motor_pwm = (motor_pwm > motor_max_pwm_)? motor_max_pwm_ : motor_pwm;
        motor_pwm = (motor_pwm < 0.0f)? 0.0f : motor_pwm;

        return motor_pwm;
    }

    double servo_curve_tilt_radians_to_pwm(double servo_tilt_angle_radians) {
        double servo_tilt_angle_degrees = radians_to_degrees(servo_tilt_angle_radians);
        return servo_tilt_angle_degrees / servo_max_tilt_angle_degrees_;
    }

};

class AttitudePIDController
{
public:
    // updated asyncronously by the caller
    std::atomic<double> angle_;
    std::atomic<double> angular_rate;
    std::atomic<double> desired_setpoint_;
    // computed by the controller
    double tilt_angle_;

    AttitudePIDController(double k_p, double k_d, double k_i, double dt)
        : angle_{0.0f}, angular_rate{0.0f}, desired_setpoint_{0.0f}, tilt_angle_(0.0f), 
            k_p_(k_p), k_d_(k_d), k_i_(k_i), integrated_error_(0.0f), dt_(dt)
            {
                // Safety check
                if (k_p == NAN || k_d == NAN || k_i == NAN) {
                    RCLCPP_ERROR(rclcpp::get_logger("attitude_pid_controller"), "Invalid PID gains provided.");
                    throw std::runtime_error("Gains vector invalid");
                }

                // Safety check
                if (dt <= 0.0f || dt == NAN) {
                    RCLCPP_ERROR(rclcpp::get_logger("attitude_pid_controller"), "Invalid time step provided.");
                    throw std::runtime_error("Time step invalid");
                }

                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_p: %f", k_p);
                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_d: %f", k_d);
                RCLCPP_INFO(rclcpp::get_logger("attitude_pid_controller"), "gains k_i: %f", k_i);
            }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    double compute() {
        // Update the integrated error
        integrated_error_ = integrated_error_ + (desired_setpoint_ - angle_) * dt_; 

        // Compute control input
        double dot_product = angle_ * k_p_ + angular_rate * k_d_;
        tilt_angle_ = -dot_product + integrated_error_ * k_i_;
        return tilt_angle_;
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
class BaselinePIDController : public rclcpp::Node
{
public:
	explicit BaselinePIDController() : Node("baseline_pid_controller"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

    attitude_subscriber_{this->create_subscription<VehicleAttitude>(
        CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_,
        [this](const VehicleAttitude::SharedPtr msg) {
            auto q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            EulerAngle euler = quaternion_to_euler_radians(q);
        
            if (roll_controller_) roll_controller_->angle_.store(euler.roll);
            if (pitch_controller_) pitch_controller_->angle_.store(euler.pitch);
            if (yaw_controller_) yaw_controller_->angle_.store(euler.yaw);
        }
    )},
    angular_rate_subscriber_{this->create_subscription<VehicleAngularVelocity>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const VehicleAngularVelocity::SharedPtr msg) {
            if (roll_controller_) roll_controller_->angular_rate.store(msg->xyz[0]);
            if (pitch_controller_) pitch_controller_->angular_rate.store(msg->xyz[1]);
            if (yaw_controller_) yaw_controller_->angular_rate.store(msg->xyz[2]);
        }
    )},
    attitude_setpoint_subscriber_{this->create_subscription<Vector3Stamped>(
        CONTROLLER_INPUT_ATTITUDE_SETPOINT_TOPIC, qos_,
        [this](const Vector3Stamped::SharedPtr msg) {
            if (msg->vector.x == NAN || msg->vector.y == NAN || msg->vector.z == NAN) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN in setpoint message.");
                return;
            }
            if (msg->vector.x < -M_PI_2 || msg->vector.y < -M_PI_2 || msg->vector.z < -M_PI ||
                msg->vector.x > M_PI_2  || msg->vector.y > M_PI_2  || msg->vector.z > M_PI
            ) {
                RCLCPP_ERROR(this->get_logger(), "Received out of range setpoint message.");
                return;
            }

            if (roll_controller_) roll_controller_->desired_setpoint_.store(msg->vector.x);
            if (pitch_controller_) pitch_controller_->desired_setpoint_.store(msg->vector.y);
            if (yaw_controller_) yaw_controller_->desired_setpoint_.store(msg->vector.z);
        }
    )},
    position_setpoint_subscriber_{this->create_subscription<Vector3Stamped>(
        CONTROLLER_INPUT_POSITION_SETPOINT_TOPIC, qos_,
        [this](const Vector3Stamped::SharedPtr msg) {
            if (msg->vector.x == NAN || msg->vector.y == NAN || msg->vector.z == NAN) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN in setpoint message.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Received position setpoint: x: %f, y: %f, z: %f",
                msg->vector.x, msg->vector.y, msg->vector.z);
        }
    )},
    servo_tilt_angle_publisher_{this->create_publisher<ActuatorServos>(
        CONTROLLER_OUTPUT_SERVO_PWM_TOPIC, qos_
    )},
    motor_thrust_publisher_{this->create_publisher<ActuatorMotors>(
        CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC, qos_
    )},
    controller_debug_publisher_{this->create_publisher<ControllerDebug>(
        CONTROLLER_DEBUG_TOPIC, qos_
    )},
    allocator_debug_publisher_{this->create_publisher<AllocatorDebug>(
        ALLOCATOR_DEBUG_TOPIC, qos_
    )},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        [this](const FlightMode::SharedPtr msg) {
            flight_mode_.store(msg->flight_mode);
        }
    )}
    {
        this->declare_parameter<double>(CONTROLLER_FREQUENCY_HERTZ_PARAM);
        double controllers_freq = this->get_parameter(CONTROLLER_FREQUENCY_HERTZ_PARAM).as_double();
        if (controllers_freq <= 0.0f || controllers_freq == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read controller frequency correctly.");
            throw std::runtime_error("Controller frequency invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Controllers freq: %f", controllers_freq);
        double controllers_dt = 1.0 / controllers_freq;

        this->declare_parameter<double>(CONTROLLER_DEFAULT_MOTOR_PWM);
        default_motor_pwm_ = this->get_parameter(CONTROLLER_DEFAULT_MOTOR_PWM).as_double();
        if (default_motor_pwm_ < 0.0f || default_motor_pwm_ > 1.0f || default_motor_pwm_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read motor thrust correctly.");
            throw std::runtime_error("Motor thrust invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Default motor thrust pwm: %f", default_motor_pwm_);


        this->declare_parameter<double>(GRAVITATIONAL_ACCELERATION);
        this->declare_parameter<double>(CONTROLLER_THRUST_CURVE_M_PARAM);
        this->declare_parameter<double>(CONTROLLER_THRUST_CURVE_B_PARAM);
        this->declare_parameter<double>(CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM);
        this->declare_parameter<double>(CONTROLLER_MOTOR_MAX_PWM_PARAM);
        double thrust_curve_m = this->get_parameter(CONTROLLER_THRUST_CURVE_M_PARAM).as_double();
        double thrust_curve_b = this->get_parameter(CONTROLLER_THRUST_CURVE_B_PARAM).as_double();
        double g               = this->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        double servo_max_tilt_angle_degrees = this->get_parameter(CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM).as_double();
        double motor_max_pwm   = this->get_parameter(CONTROLLER_MOTOR_MAX_PWM_PARAM).as_double();

        allocator_ = std::make_unique<Allocator>(
            thrust_curve_m, thrust_curve_b, g, 
            servo_max_tilt_angle_degrees, motor_max_pwm
        );

        this->declare_parameter<bool>(CONTROLLER_ROLL_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_ROLL_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Roll Controller is active");
            this->declare_parameter<double>(CONTROLLER_ROLL_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_ROLL_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_ROLL_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_ROLL_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_ROLL_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_ROLL_K_I_PARAM).as_double();

            roll_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Roll Controller is not active");
            roll_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_PITCH_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_PITCH_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Pitch Controller is active");
            this->declare_parameter<double>(CONTROLLER_PITCH_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_PITCH_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_PITCH_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_PITCH_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_PITCH_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_PITCH_K_I_PARAM).as_double();

            pitch_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Pitch Controller is not active");
            pitch_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_YAW_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_YAW_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Yaw Controller is active");
            this->declare_parameter<double>(CONTROLLER_YAW_K_P_PARAM);
            this->declare_parameter<double>(CONTROLLER_YAW_K_D_PARAM);
            this->declare_parameter<double>(CONTROLLER_YAW_K_I_PARAM);

            double k_p = this->get_parameter(CONTROLLER_YAW_K_P_PARAM).as_double();
            double k_d = this->get_parameter(CONTROLLER_YAW_K_D_PARAM).as_double();
            double k_i = this->get_parameter(CONTROLLER_YAW_K_I_PARAM).as_double();

            yaw_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Yaw Controller is not active");
            yaw_controller_ = std::nullopt;
        }


        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(controllers_dt), 
            std::bind(&BaselinePIDController::controller_callback, this)
        );
	}

private:
    rmw_qos_profile_t qos_profile_;
    rclcpp::QoS qos_;

    //!< Time variables
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Publishers and Subscribers
	rclcpp::Subscription<VehicleAttitude>::SharedPtr        attitude_subscriber_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr angular_rate_subscriber_;
	rclcpp::Subscription<Vector3Stamped>::SharedPtr         attitude_setpoint_subscriber_;
	rclcpp::Subscription<Vector3Stamped>::SharedPtr         position_setpoint_subscriber_;
	rclcpp::Publisher<ActuatorServos>::SharedPtr    servo_tilt_angle_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr    motor_thrust_publisher_;

    rclcpp::Publisher<ControllerDebug>::SharedPtr controller_debug_publisher_;
    rclcpp::Publisher<AllocatorDebug>::SharedPtr  allocator_debug_publisher_;

    // radians, radians per second
    std::optional<AttitudePIDController> roll_controller_;
    std::optional<AttitudePIDController> pitch_controller_;
    std::optional<AttitudePIDController> yaw_controller_;

    std::unique_ptr<Allocator> allocator_;

    double default_motor_pwm_;

	//!< Auxiliary functions
    void controller_callback();
    void publish_controller_debug();
    void publish_allocator_debug(
        const double inner_servo_tilt_angle_radians,
        const double outer_servo_tilt_angle_radians,
        const double inner_servo_pwm,
        const double outer_servo_pwm,

        const double delta_motor_pwm,
        const double average_motor_thrust_newtons,
        const double upwards_motor_pwm,
        const double downwards_motor_pwm
    );

    void publish_servo_pwm(const double inner_servo_pwm, const double outer_servo_pwm);
    void publish_motor_pwm(const double upwards_motor_pwm, const double downwards_motor_pwm);

    std::atomic<uint8_t> flight_mode_;
    rclcpp::Subscription<FlightMode>::SharedPtr flight_mode_get_subscriber_;
	void response_flight_mode_callback(const std::shared_ptr<FlightMode> response);
};

/**
 * @brief Callback function for the controller
 */
void BaselinePIDController::controller_callback()
{
    // only run controller when in mission
    if (flight_mode_.load() == FlightMode::IN_MISSION) {
        double inner_servo_tilt_angle_radians = (roll_controller_)  ? roll_controller_->compute() : 0.0f;
        double outer_servo_tilt_angle_radians = (pitch_controller_) ? pitch_controller_->compute() : 0.0f;
        double delta_motor_pwm = (yaw_controller_) ? yaw_controller_->compute() : 0.0f;
        double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(default_motor_pwm_);

        Allocator::ServoAllocatorOutput servo_output = allocator_->compute_servo_allocation(
            inner_servo_tilt_angle_radians,
            outer_servo_tilt_angle_radians
        );

        publish_servo_pwm(
            servo_output.inner_servo_pwm,
            servo_output.outer_servo_pwm
        );

        // Allocate motor thrust based on the computed torque
        Allocator::MotorAllocatorOutput motor_output = allocator_->compute_motor_allocation(
            delta_motor_pwm, 
            average_motor_thrust_newtons
        );

        // Publish the motor thrust
        publish_motor_pwm(
            motor_output.upwards_motor_pwm, 
            motor_output.downwards_motor_pwm
        );

        // Publish the controller debug information
        publish_controller_debug();
        publish_allocator_debug(
            inner_servo_tilt_angle_radians,
            outer_servo_tilt_angle_radians,
            servo_output.inner_servo_pwm,
            servo_output.outer_servo_pwm,

            delta_motor_pwm,
            average_motor_thrust_newtons,
            motor_output.upwards_motor_pwm,
            motor_output.downwards_motor_pwm
        );
    }
}

void BaselinePIDController::publish_controller_debug() {
    ControllerDebug msg{};

    if (roll_controller_) {
        msg.roll_angle = radians_to_degrees(roll_controller_->angle_);
        msg.roll_angular_velocity = radians_to_degrees(roll_controller_->angular_rate);
        msg.roll_angle_setpoint = radians_to_degrees(roll_controller_->desired_setpoint_);
        msg.roll_inner_servo_tilt_angle = radians_to_degrees(roll_controller_->tilt_angle_);
    }

    if (pitch_controller_) {
        msg.pitch_angle = radians_to_degrees(pitch_controller_->angle_);
        msg.pitch_angular_velocity = radians_to_degrees(pitch_controller_->angular_rate);
        msg.pitch_angle_setpoint = radians_to_degrees(pitch_controller_->desired_setpoint_);
        msg.pitch_outer_servo_tilt_angle = radians_to_degrees(pitch_controller_->tilt_angle_);
    }

    if (yaw_controller_) {
        msg.yaw_angle = radians_to_degrees(yaw_controller_->angle_);
        msg.yaw_angular_velocity = radians_to_degrees(yaw_controller_->angular_rate);
        msg.yaw_angle_setpoint = radians_to_degrees(yaw_controller_->desired_setpoint_);
        msg.yaw_delta_motor_pwm = radians_to_degrees(yaw_controller_->tilt_angle_);
    }

    msg.stamp = this->get_clock()->now();
    controller_debug_publisher_->publish(msg);
}

void BaselinePIDController::publish_allocator_debug(
    const double inner_servo_tilt_angle_radians,
    const double outer_servo_tilt_angle_radians,
    const double inner_servo_pwm,
    const double outer_servo_pwm,

    const double delta_motor_pwm,
    const double average_motor_thrust_newtons,
    const double upwards_motor_pwm,
    const double downwards_motor_pwm
) {
    AllocatorDebug msg{};
    msg.stamp = this->get_clock()->now();

    msg.servo_inner_tilt_angle_degrees = radians_to_degrees(inner_servo_tilt_angle_radians);
    msg.servo_outer_tilt_angle_degrees = radians_to_degrees(outer_servo_tilt_angle_radians);
    msg.servo_inner_pwm = inner_servo_pwm;
    msg.servo_outer_pwm = outer_servo_pwm;

    msg.motor_delta_pwm = delta_motor_pwm;
    msg.motor_average_pwm = allocator_->motor_thrust_curve_newtons_to_pwm(average_motor_thrust_newtons);
    msg.motor_upwards_pwm = upwards_motor_pwm;
    msg.motor_downwards_pwm = downwards_motor_pwm;

    allocator_debug_publisher_->publish(msg);
}

void BaselinePIDController::publish_servo_pwm(const double inner_servo_pwm, const double outer_servo_pwm)
{
    ActuatorServos msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.control[1] = inner_servo_pwm;
    msg.control[0] = outer_servo_pwm;
    servo_tilt_angle_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void BaselinePIDController::publish_motor_pwm(const double upwards_motor_pwm, const double downwards_motor_pwm)
{
	ActuatorMotors msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.control[1] = upwards_motor_pwm;
	msg.control[0] = downwards_motor_pwm;
	motor_thrust_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard baseline pid controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BaselinePIDController>());

	rclcpp::shutdown();
	return 0;
}
