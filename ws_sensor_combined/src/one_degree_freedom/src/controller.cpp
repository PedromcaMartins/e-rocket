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
        float inner_servo_pwm;
        float outer_servo_pwm;
    };

    struct MotorAllocatorOutput {
        float upwards_motor_pwm;
        float downwards_motor_pwm;
    };

    Allocator(float thrust_curve_m, float thrust_curve_b, float g, 
        float servo_max_tilt_angle_degrees, float motor_max_pwm
    )   : thrust_curve_m_(thrust_curve_m), thrust_curve_b_(thrust_curve_b), g_(g), 
            servo_max_tilt_angle_degrees_(servo_max_tilt_angle_degrees), motor_max_pwm_(motor_max_pwm)
            {
                if (thrust_curve_m == NAN || thrust_curve_b == NAN || g <= 0.0f || g == NAN ||
                    servo_max_tilt_angle_degrees <= 0.0f || servo_max_tilt_angle_degrees > 90.0f ||
                    servo_max_tilt_angle_degrees == NAN || motor_max_pwm < 0.0f || motor_max_pwm > 1.0 || motor_max_pwm == NAN
                ) {
                    RCLCPP_ERROR(rclcpp::get_logger("Allocator"), "Invalid parameters for allocator.");
                    throw std::runtime_error("Allocator parameters invalid");
                }
                RCLCPP_INFO(rclcpp::get_logger("Allocator"), "Thrust curve: x * %f + %f", thrust_curve_m_, thrust_curve_b_);
                RCLCPP_INFO(rclcpp::get_logger("Allocator"), "Gravitational acceleration: %f", g_);
                RCLCPP_INFO(rclcpp::get_logger("Allocator"), "Servo max tilt angle degrees: %f", servo_max_tilt_angle_degrees_);
                RCLCPP_INFO(rclcpp::get_logger("Allocator"), "Motor max pwm: %f", motor_max_pwm_);
            }

    ServoAllocatorOutput compute_servo_allocation(
        float inner_servo_tilt_angle_radians, 
        float outer_servo_tilt_angle_radians
    ) {
        ServoAllocatorOutput output;

        float inner_servo_pwm = servo_curve_tilt_radians_to_pwm(inner_servo_tilt_angle_radians);
        float outer_servo_pwm = servo_curve_tilt_radians_to_pwm(outer_servo_tilt_angle_radians);

        output.inner_servo_pwm = limit_range_servo_pwm(inner_servo_pwm);
        output.outer_servo_pwm = limit_range_servo_pwm(outer_servo_pwm);

        return output;
    }

    MotorAllocatorOutput compute_motor_allocation(
        float delta_motor_pwm, 
        float average_motor_thrust_newtons
    ) {
        MotorAllocatorOutput output;

        float average_motor_pwm = motor_thrust_curve_newtons_to_pwm(average_motor_thrust_newtons);
        float upwards_motor_pwm = average_motor_pwm - delta_motor_pwm / 2.0f;
        float downwards_motor_pwm = average_motor_pwm + delta_motor_pwm / 2.0f;

        output.upwards_motor_pwm = limit_range_motor_pwm(upwards_motor_pwm);
        output.downwards_motor_pwm = limit_range_motor_pwm(downwards_motor_pwm);

        return output;
    }

    float motor_thrust_curve_pwm_to_newtons(float motor_pwm) {
        return (motor_pwm * thrust_curve_m_ + thrust_curve_b_) / 1000.0f * g_;
    }

    float motor_thrust_curve_newtons_to_pwm(float thrust_newtons) {
        return ((thrust_newtons * 1000.0f) / g_ - thrust_curve_b_) / thrust_curve_m_;
    }

private: 
    float thrust_curve_m_;
    float thrust_curve_b_;
    float g_;

    float servo_max_tilt_angle_degrees_;
    float motor_max_pwm_;

    float limit_range_servo_pwm(float servo_pwm) {
        servo_pwm = (servo_pwm > 1.0f)? 1.0f : servo_pwm;
        servo_pwm = (servo_pwm < -1.0f)? -1.0f : servo_pwm;

        return servo_pwm;
    }

    float limit_range_motor_pwm(float motor_pwm) {
        motor_pwm = (motor_pwm > motor_max_pwm_)? motor_max_pwm_ : motor_pwm;
        motor_pwm = (motor_pwm < 0.0f)? 0.0f : motor_pwm;

        return motor_pwm;
    }

    float servo_curve_tilt_radians_to_pwm(float servo_tilt_angle_radians) {
        float servo_tilt_angle_degrees = radians_to_degrees(servo_tilt_angle_radians);
        return servo_tilt_angle_degrees / servo_max_tilt_angle_degrees_;
    }

};

class PIDController
{
public:
    // updated asyncronously by the caller
    std::atomic<float> measurement_;
    std::atomic<float> measurement_derivative_;
    std::atomic<float> desired_setpoint_;
    float computed_output_;

    PIDController(float k_p, float k_d, float k_i, float dt)
        : measurement_{0.0f}, measurement_derivative_{0.0f}, desired_setpoint_{0.0f}, computed_output_(0.0f), 
            k_p_(k_p), k_d_(k_d), k_i_(k_i), integrated_error_(0.0f), dt_(dt)
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

                RCLCPP_INFO(rclcpp::get_logger("PIDController"), "gains k_p: %f", k_p);
                RCLCPP_INFO(rclcpp::get_logger("PIDController"), "gains k_d: %f", k_d);
                RCLCPP_INFO(rclcpp::get_logger("PIDController"), "gains k_i: %f", k_i);
            }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    float compute() {
        // Update the integrated error
        integrated_error_ = integrated_error_ + (desired_setpoint_ - measurement_) * dt_; 

        // Compute control input
        float dot_product = measurement_ * k_p_ + measurement_derivative_ * k_d_;
        computed_output_ = -dot_product + integrated_error_ * k_i_;
        return computed_output_;
    }

private:
    const float k_p_;
    const float k_d_;
    const float k_i_;
    float integrated_error_;
    const float dt_;
};

/**
 * @brief Node that runs the controller for a 1-degree-of-freedom system
 */
class BaselinePIDController : public rclcpp::Node
{
public:
	explicit BaselinePIDController() : Node("controller_decoupled"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

    attitude_subscriber_{this->create_subscription<VehicleAttitude>(
        CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_,
        [this](const VehicleAttitude::SharedPtr msg) {
            auto q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            EulerAngle euler = quaternion_to_euler_radians(q);
        
            if (roll_controller_) roll_controller_->measurement_.store(euler.roll);
            if (pitch_controller_) pitch_controller_->measurement_.store(euler.pitch);
            if (yaw_controller_) yaw_controller_->measurement_.store(euler.yaw);
        }
    )},
    angular_rate_subscriber_{this->create_subscription<VehicleAngularVelocity>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const VehicleAngularVelocity::SharedPtr msg) {
            if (roll_controller_) roll_controller_->measurement_derivative_.store(msg->xyz[0]);
            if (pitch_controller_) pitch_controller_->measurement_derivative_.store(msg->xyz[1]);
            if (yaw_controller_) yaw_controller_->measurement_derivative_.store(msg->xyz[2]);
        }
    )},
    setpoint_attitude_subscriber_{this->create_subscription<Vector3Stamped>(
        CONTROLLER_INPUT_SETPOINT_ATTITUDE_TOPIC, qos_,
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
        this->declare_parameter<float>(CONTROLLER_FREQUENCY_HERTZ_PARAM);
        float controllers_freq = this->get_parameter(CONTROLLER_FREQUENCY_HERTZ_PARAM).as_double();
        if (controllers_freq <= 0.0f || controllers_freq == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read controller frequency correctly.");
            throw std::runtime_error("Controller frequency invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Controllers freq: %f", controllers_freq);
        float controllers_dt = 1.0 / controllers_freq;

        this->declare_parameter<float>(CONTROLLER_DEFAULT_MOTOR_PWM);
        default_motor_pwm_ = this->get_parameter(CONTROLLER_DEFAULT_MOTOR_PWM).as_double();
        if (default_motor_pwm_ < 0.0f || default_motor_pwm_ > 1.0f || default_motor_pwm_ == NAN) {
            RCLCPP_ERROR(this->get_logger(), "Could not read motor thrust correctly.");
            throw std::runtime_error("Motor thrust invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Default motor thrust pwm: %f", default_motor_pwm_);


        this->declare_parameter<float>(GRAVITATIONAL_ACCELERATION);
        this->declare_parameter<float>(CONTROLLER_THRUST_CURVE_M_PARAM);
        this->declare_parameter<float>(CONTROLLER_THRUST_CURVE_B_PARAM);
        this->declare_parameter<float>(CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM);
        this->declare_parameter<float>(CONTROLLER_MOTOR_MAX_PWM_PARAM);
        float thrust_curve_m = this->get_parameter(CONTROLLER_THRUST_CURVE_M_PARAM).as_double();
        float thrust_curve_b = this->get_parameter(CONTROLLER_THRUST_CURVE_B_PARAM).as_double();
        float g               = this->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        float servo_max_tilt_angle_degrees = this->get_parameter(CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM).as_double();
        float motor_max_pwm   = this->get_parameter(CONTROLLER_MOTOR_MAX_PWM_PARAM).as_double();

        allocator_ = std::make_unique<Allocator>(
            thrust_curve_m, thrust_curve_b, g, 
            servo_max_tilt_angle_degrees, motor_max_pwm
        );

        this->declare_parameter<bool>(CONTROLLER_ROLL_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_ROLL_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Roll is active");
            this->declare_parameter<float>(CONTROLLER_ROLL_K_P_PARAM);
            this->declare_parameter<float>(CONTROLLER_ROLL_K_D_PARAM);
            this->declare_parameter<float>(CONTROLLER_ROLL_K_I_PARAM);

            float k_p = this->get_parameter(CONTROLLER_ROLL_K_P_PARAM).as_double();
            float k_d = this->get_parameter(CONTROLLER_ROLL_K_D_PARAM).as_double();
            float k_i = this->get_parameter(CONTROLLER_ROLL_K_I_PARAM).as_double();

            roll_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Roll is not active");
            roll_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_PITCH_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_PITCH_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Pitch is active");
            this->declare_parameter<float>(CONTROLLER_PITCH_K_P_PARAM);
            this->declare_parameter<float>(CONTROLLER_PITCH_K_D_PARAM);
            this->declare_parameter<float>(CONTROLLER_PITCH_K_I_PARAM);

            float k_p = this->get_parameter(CONTROLLER_PITCH_K_P_PARAM).as_double();
            float k_d = this->get_parameter(CONTROLLER_PITCH_K_D_PARAM).as_double();
            float k_i = this->get_parameter(CONTROLLER_PITCH_K_I_PARAM).as_double();

            pitch_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Pitch is not active");
            pitch_controller_ = std::nullopt;
        }


        this->declare_parameter<bool>(CONTROLLER_YAW_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_YAW_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Yaw is active");
            this->declare_parameter<float>(CONTROLLER_YAW_K_P_PARAM);
            this->declare_parameter<float>(CONTROLLER_YAW_K_D_PARAM);
            this->declare_parameter<float>(CONTROLLER_YAW_K_I_PARAM);

            float k_p = this->get_parameter(CONTROLLER_YAW_K_P_PARAM).as_double();
            float k_d = this->get_parameter(CONTROLLER_YAW_K_D_PARAM).as_double();
            float k_i = this->get_parameter(CONTROLLER_YAW_K_I_PARAM).as_double();

            yaw_controller_.emplace(k_p, k_d, k_i, controllers_dt);
        } else {
            RCLCPP_INFO(this->get_logger(), "Yaw is not active");
            yaw_controller_ = std::nullopt;
        }


        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<float>(controllers_dt), 
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
	rclcpp::Subscription<Vector3Stamped>::SharedPtr         setpoint_attitude_subscriber_;
	rclcpp::Publisher<ActuatorServos>::SharedPtr    servo_tilt_angle_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr    motor_thrust_publisher_;

    rclcpp::Publisher<ControllerDebug>::SharedPtr controller_debug_publisher_;
    rclcpp::Publisher<AllocatorDebug>::SharedPtr  allocator_debug_publisher_;

    // radians, radians per second
    std::optional<PIDController> roll_controller_;
    std::optional<PIDController> pitch_controller_;
    std::optional<PIDController> yaw_controller_;

    std::unique_ptr<Allocator> allocator_;

    float default_motor_pwm_;

	//!< Auxiliary functions
    void controller_callback();
    void publish_controller_debug();
    void publish_allocator_debug(
        const float inner_servo_tilt_angle_radians,
        const float outer_servo_tilt_angle_radians,
        const float inner_servo_pwm,
        const float outer_servo_pwm,

        const float delta_motor_pwm,
        const float average_motor_thrust_newtons,
        const float upwards_motor_pwm,
        const float downwards_motor_pwm
    );

    void publish_servo_pwm(const float inner_servo_pwm, const float outer_servo_pwm);
    void publish_motor_pwm(const float upwards_motor_pwm, const float downwards_motor_pwm);

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
        float inner_servo_tilt_angle_radians = (roll_controller_)  ? roll_controller_->compute() : 0.0f;
        float outer_servo_tilt_angle_radians = (pitch_controller_) ? pitch_controller_->compute() : 0.0f;
        float delta_motor_pwm = (yaw_controller_) ? yaw_controller_->compute() : 0.0f;
        float average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(default_motor_pwm_);

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
        msg.roll_angle = radians_to_degrees(roll_controller_->measurement_);
        msg.roll_angular_velocity = radians_to_degrees(roll_controller_->measurement_derivative_);
        msg.roll_angle_setpoint = radians_to_degrees(roll_controller_->desired_setpoint_);
        msg.roll_inner_servo_tilt_angle = radians_to_degrees(roll_controller_->computed_output_);
    }

    if (pitch_controller_) {
        msg.pitch_angle = radians_to_degrees(pitch_controller_->measurement_);
        msg.pitch_angular_velocity = radians_to_degrees(pitch_controller_->measurement_derivative_);
        msg.pitch_angle_setpoint = radians_to_degrees(pitch_controller_->desired_setpoint_);
        msg.pitch_outer_servo_tilt_angle = radians_to_degrees(pitch_controller_->computed_output_);
    }

    if (yaw_controller_) {
        msg.yaw_angle = radians_to_degrees(yaw_controller_->measurement_);
        msg.yaw_angular_velocity = radians_to_degrees(yaw_controller_->measurement_derivative_);
        msg.yaw_angle_setpoint = radians_to_degrees(yaw_controller_->desired_setpoint_);
        msg.yaw_delta_motor_pwm = radians_to_degrees(yaw_controller_->computed_output_);
    }

    msg.stamp = this->get_clock()->now();
    controller_debug_publisher_->publish(msg);
}

void BaselinePIDController::publish_allocator_debug(
    const float inner_servo_tilt_angle_radians,
    const float outer_servo_tilt_angle_radians,
    const float inner_servo_pwm,
    const float outer_servo_pwm,

    const float delta_motor_pwm,
    const float average_motor_thrust_newtons,
    const float upwards_motor_pwm,
    const float downwards_motor_pwm
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

void BaselinePIDController::publish_servo_pwm(const float inner_servo_pwm, const float outer_servo_pwm)
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
void BaselinePIDController::publish_motor_pwm(const float upwards_motor_pwm, const float downwards_motor_pwm)
{
	ActuatorMotors msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.control[1] = upwards_motor_pwm;
	msg.control[0] = downwards_motor_pwm;
	motor_thrust_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard controller decoupled node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BaselinePIDController>());

	rclcpp::shutdown();
	return 0;
}
