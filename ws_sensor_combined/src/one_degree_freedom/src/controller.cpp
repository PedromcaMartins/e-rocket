#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <one_degree_freedom/msg/trajectory_setpoint.hpp>
#include <one_degree_freedom/msg/attitude_controller_debug.hpp>
#include <one_degree_freedom/msg/position_controller_debug.hpp>
#include <one_degree_freedom/msg/allocator_debug.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <one_degree_freedom/controller/allocator.hpp>
#include <one_degree_freedom/controller/impls/attitude_pid.hpp>
#include <one_degree_freedom/controller/impls/position_pid.hpp>

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
        
            if (roll_controller_) roll_controller_->angle_ = euler.roll;
            if (pitch_controller_) pitch_controller_->angle_ = euler.pitch;
            if (yaw_controller_) yaw_controller_->angle_ = euler.yaw;
        }
    )},
    angular_rate_subscriber_{this->create_subscription<VehicleAngularVelocity>(
        CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_,
        [this](const VehicleAngularVelocity::SharedPtr msg) {
            if (roll_controller_) roll_controller_->angular_rate = msg->xyz[0];
            if (pitch_controller_) pitch_controller_->angular_rate = msg->xyz[1];
            if (yaw_controller_) yaw_controller_->angular_rate = msg->xyz[2];
        }
    )},
    local_position_subscriber_{this->create_subscription<VehicleLocalPosition>(
        CONTROLLER_INPUT_LOCAL_POSITION_TOPIC, qos_,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            if (position_controller_) {
                position_controller_->position_[0] = msg->x;
                position_controller_->position_[1] = msg->y;
                position_controller_->position_[2] = msg->z;

                position_controller_->velocity_[0] = msg->vx;
                position_controller_->velocity_[1] = msg->vy;
                position_controller_->velocity_[2] = msg->vz;

                position_controller_->acceleration_[0] = msg->ax;
                position_controller_->acceleration_[1] = msg->ay;
                position_controller_->acceleration_[2] = msg->az;
            }
        }
    )},
    attitude_setpoint_subscriber_{this->create_subscription<Vector3Stamped>(
        CONTROLLER_INPUT_ATTITUDE_SETPOINT_TOPIC, qos_,
        [this](const Vector3Stamped::SharedPtr msg) {
            if (std::isnan(msg->vector.x) || std::isnan(msg->vector.y) || std::isnan(msg->vector.z)) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN in setpoint message.");
                return;
            }
            if (msg->vector.x < -M_PI_2 || msg->vector.y < -M_PI_2 || msg->vector.z < -M_PI ||
                msg->vector.x > M_PI_2  || msg->vector.y > M_PI_2  || msg->vector.z > M_PI
            ) {
                RCLCPP_ERROR(this->get_logger(), "Received out of range setpoint message.");
                return;
            }

            if (roll_controller_) roll_controller_->desired_setpoint_ = msg->vector.x;
            if (pitch_controller_) pitch_controller_->desired_setpoint_ = msg->vector.y;
            if (yaw_controller_) yaw_controller_->desired_setpoint_ = msg->vector.z;
            if (position_controller_) position_controller_->yaw_angle_setpoint_ = msg->vector.z;
        }
    )},
    translation_position_setpoint_subscriber_{this->create_subscription<Vector3Stamped>(
        CONTROLLER_INPUT_TRANSLATION_POSITION_SETPOINT_TOPIC, qos_,
        [this](const Vector3Stamped::SharedPtr msg) {
            if (std::isnan(msg->vector.x) || std::isnan(msg->vector.y) || std::isnan(msg->vector.z)) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN in setpoint message.");
                return;
            }

            if (position_controller_) {
                position_controller_->position_setpoint_[0] = position_controller_->position_setpoint_[0] + msg->vector.x;
                position_controller_->position_setpoint_[1] = position_controller_->position_setpoint_[1] + msg->vector.y;
                position_controller_->position_setpoint_[2] = position_controller_->position_setpoint_[2] + msg->vector.z;
            }
        }
    )},
    trajectory_setpoint_subscriber_{this->create_subscription<TrajectorySetpoint>(
        CONTROLLER_INPUT_TRAJECTORY_SETPOINT_TOPIC, qos_,
        [this](const TrajectorySetpoint::SharedPtr msg) {
            if (std::isnan(msg->position[0]) || std::isnan(msg->position[1]) || std::isnan(msg->position[2]) ||
                std::isnan(msg->velocity[0]) || std::isnan(msg->velocity[1]) || std::isnan(msg->velocity[2]) ||
                std::isnan(msg->acceleration[0]) || std::isnan(msg->acceleration[1]) || std::isnan(msg->acceleration[2])) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN in setpoint message.");
                return;
            }

            if (position_controller_) {
                position_controller_->position_setpoint_[0] = msg->position[0];
                position_controller_->position_setpoint_[1] = msg->position[1];
                position_controller_->position_setpoint_[2] = msg->position[2];

                position_controller_->velocity_setpoint_[0] = msg->velocity[0];
                position_controller_->velocity_setpoint_[1] = msg->velocity[1];
                position_controller_->velocity_setpoint_[2] = msg->velocity[2];

                position_controller_->acceleration_setpoint_[0] = msg->acceleration[0];
                position_controller_->acceleration_setpoint_[1] = msg->acceleration[1];
                position_controller_->acceleration_setpoint_[2] = msg->acceleration[2];
            }
        }
    )},
    servo_tilt_angle_publisher_{this->create_publisher<ActuatorServos>(
        CONTROLLER_OUTPUT_SERVO_PWM_TOPIC, qos_
    )},
    motor_thrust_publisher_{this->create_publisher<ActuatorMotors>(
        CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC, qos_
    )},
    attitude_controller_debug_publisher_{this->create_publisher<AttitudeControllerDebug>(
        CONTROLLER_ATTITUDE_DEBUG_TOPIC, qos_
    )},
    position_controller_debug_publisher_{this->create_publisher<PositionControllerDebug>(
        CONTROLLER_POSITION_DEBUG_TOPIC, qos_
    )},
    allocator_debug_publisher_{this->create_publisher<AllocatorDebug>(
        ALLOCATOR_DEBUG_TOPIC, qos_
    )},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        [this](const FlightMode::SharedPtr msg) {
            flight_mode_ = msg->flight_mode;
        }
    )}
    {
        this->declare_parameter<bool>(CONTROLLER_OUTPUT_MOTOR_ACTIVE_PARAM);
        this->declare_parameter<bool>(CONTROLLER_OUTPUT_SERVO_ACTIVE_PARAM);
        motor_active_ = this->get_parameter(CONTROLLER_OUTPUT_MOTOR_ACTIVE_PARAM).as_bool();
        servo_active_ = this->get_parameter(CONTROLLER_OUTPUT_SERVO_ACTIVE_PARAM).as_bool();

        RCLCPP_INFO(this->get_logger(), "Motor %s; Servo %s", 
            (motor_active_)? "active" : "off",
            (servo_active_)? "active" : "off"
        );

        this->declare_parameter<double>(GRAVITATIONAL_ACCELERATION);
        this->declare_parameter<double>(MASS_OF_SYSTEM);
        double g = this->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        double mass = this->get_parameter(MASS_OF_SYSTEM).as_double();

        this->declare_parameter<double>(CONTROLLER_FREQUENCY_HERTZ_PARAM);
        double controllers_freq = this->get_parameter(CONTROLLER_FREQUENCY_HERTZ_PARAM).as_double();
        if (controllers_freq <= 0.0f || std::isnan(controllers_freq)) {
            RCLCPP_ERROR(this->get_logger(), "Could not read controller frequency correctly.");
            throw std::runtime_error("Controller frequency invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Controllers freq: %f", controllers_freq);
        double controllers_dt = 1.0 / controllers_freq;

        this->declare_parameter<double>(CONTROLLER_DEFAULT_MOTOR_PWM);
        default_motor_pwm_ = this->get_parameter(CONTROLLER_DEFAULT_MOTOR_PWM).as_double();
        if (default_motor_pwm_ < 0.0f || default_motor_pwm_ > 1.0f || std::isnan(default_motor_pwm_)) {
            RCLCPP_ERROR(this->get_logger(), "Could not read motor thrust correctly.");
            throw std::runtime_error("Motor thrust invalid");
        }
        RCLCPP_INFO(this->get_logger(), "Default motor thrust pwm: %f", default_motor_pwm_);


        this->declare_parameter<double>(CONTROLLER_THRUST_CURVE_M_PARAM);
        this->declare_parameter<double>(CONTROLLER_THRUST_CURVE_B_PARAM);
        this->declare_parameter<double>(CONTROLLER_SERVO_MAX_TILT_ANGLE_PARAM);
        this->declare_parameter<double>(CONTROLLER_MOTOR_MAX_PWM_PARAM);
        double thrust_curve_m = this->get_parameter(CONTROLLER_THRUST_CURVE_M_PARAM).as_double();
        double thrust_curve_b = this->get_parameter(CONTROLLER_THRUST_CURVE_B_PARAM).as_double();
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


        this->declare_parameter<bool>(CONTROLLER_POSITION_ACTIVE_PARAM);
        if (this->get_parameter(CONTROLLER_POSITION_ACTIVE_PARAM).as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Position Controller is active");
            this->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_P_PARAM);
            this->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_D_PARAM);
            this->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_K_I_PARAM);
            this->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_MIN_OUTPUT_PARAM);
            this->declare_parameter<std::vector<double>>(CONTROLLER_POSITION_MAX_OUTPUT_PARAM);

            std::vector<double> k_p = this->get_parameter(CONTROLLER_POSITION_K_P_PARAM).as_double_array();
            std::vector<double> k_d = this->get_parameter(CONTROLLER_POSITION_K_D_PARAM).as_double_array();
            std::vector<double> k_i = this->get_parameter(CONTROLLER_POSITION_K_I_PARAM).as_double_array();
            std::vector<double> min_output = this->get_parameter(CONTROLLER_POSITION_MIN_OUTPUT_PARAM).as_double_array();
            std::vector<double> max_output = this->get_parameter(CONTROLLER_POSITION_MAX_OUTPUT_PARAM).as_double_array();

            position_controller_.emplace(
                mass, 
                g,
                Eigen::Vector3d {k_p[0], k_p[1], k_p[2]}, 
                Eigen::Vector3d {k_d[0], k_d[1], k_d[2]}, 
                Eigen::Vector3d {k_i[0], k_i[1], k_i[2]},
                Eigen::Vector3d { 1.0f,   1.0f,   1.0f}, // No feedforward for position controller
                Eigen::Vector3d {min_output[0], min_output[1], min_output[2]},
                Eigen::Vector3d {max_output[0], max_output[1], max_output[2]},
                controllers_dt
            );
        } else {
            RCLCPP_INFO(this->get_logger(), "Position Controller is not active");
            position_controller_ = std::nullopt;
        }

        if (position_controller_ && (!roll_controller_ || !pitch_controller_ || !yaw_controller_)) {
            RCLCPP_ERROR(this->get_logger(), "Position controller requires all attitude controllers to be active.");
            throw std::runtime_error("Position controller requires all attitude controllers to be active.");
        }

        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(controllers_dt), 
            std::bind(&BaselinePIDController::controller_callback, this)
        );

        publish_servo_pwm(
            0.0f,
            0.0f
        );

        publish_motor_pwm(
            NAN,
            NAN
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
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr   local_position_subscriber_;
	rclcpp::Subscription<Vector3Stamped>::SharedPtr         attitude_setpoint_subscriber_;
	rclcpp::Subscription<Vector3Stamped>::SharedPtr         translation_position_setpoint_subscriber_;
	rclcpp::Subscription<TrajectorySetpoint>::SharedPtr     trajectory_setpoint_subscriber_;
	rclcpp::Publisher<ActuatorServos>::SharedPtr    servo_tilt_angle_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr    motor_thrust_publisher_;

    rclcpp::Publisher<AttitudeControllerDebug>::SharedPtr attitude_controller_debug_publisher_;
    rclcpp::Publisher<PositionControllerDebug>::SharedPtr position_controller_debug_publisher_;
    rclcpp::Publisher<AllocatorDebug>::SharedPtr  allocator_debug_publisher_;

    // radians, radians per second
    std::optional<AttitudePIDController> roll_controller_;
    std::optional<AttitudePIDController> pitch_controller_;
    std::optional<AttitudePIDController> yaw_controller_;

    std::unique_ptr<Allocator> allocator_;

    std::optional<PositionPIDController> position_controller_;

    double default_motor_pwm_;
    bool motor_active_;
    bool servo_active_;

	//!< Auxiliary functions
    void controller_callback();
    void publish_attitude_controller_debug();
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
    void publish_position_controller_debug();

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
    if (flight_mode_ == FlightMode::ARM) {
        static rclcpp::Time t0 = this->get_clock()->now();
        rclcpp::Time now = this->get_clock()->now();

        double inner_servo_tilt_angle_radians = 0.0;
        double outer_servo_tilt_angle_radians = 0.0;

        if (now - t0 < 3.0s) {
            inner_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds()) * degrees_to_radians(30.0);
            outer_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds() + M_PI / 2.0) * degrees_to_radians(30.0);
        } else {
            inner_servo_tilt_angle_radians =  0. ;
            outer_servo_tilt_angle_radians =  0. ;
        }

        Allocator::ServoAllocatorOutput servo_output = allocator_->compute_servo_allocation(
            inner_servo_tilt_angle_radians,
            outer_servo_tilt_angle_radians
        );

        publish_servo_pwm(
            servo_output.inner_servo_pwm,
            servo_output.outer_servo_pwm
        );

        if (now - t0 > 4.0s) {
            // Needed 
            double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(0.4f);
            yaw_controller_->desired_setpoint_ = 0.0f;
            double delta_motor_pwm = yaw_controller_->compute();

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
            publish_attitude_controller_debug();
        }
    }

    // only run controller when in mission
    else if (flight_mode_ == FlightMode::IN_MISSION) {
        double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(default_motor_pwm_);

        if (position_controller_) {
            static bool mission_start = false;
            if (!mission_start) {
                mission_start = true;
                position_controller_->set_position_as_origin(
                    Eigen::Vector3d(
                        position_controller_->position_[0],
                        position_controller_->position_[1],
                        position_controller_->position_[2]
                    ),
                    yaw_controller_->angle_
                );
                RCLCPP_INFO(this->get_logger(), "Position controller origin set to current position.");
                RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f], yaw: %f", 
                    position_controller_->position_[0].load(),
                    position_controller_->position_[1].load(),
                    position_controller_->position_[2].load(),
                    yaw_controller_->angle_.load()
                );
            }

            position_controller_->compute();
            roll_controller_->desired_setpoint_ = position_controller_->desired_attitude_[0];
            pitch_controller_->desired_setpoint_ = position_controller_->desired_attitude_[1];
            yaw_controller_->desired_setpoint_ = position_controller_->desired_attitude_[2];

            average_motor_thrust_newtons = position_controller_->desired_thrust_;

            publish_position_controller_debug();
        }

        double inner_servo_tilt_angle_radians = (roll_controller_)  ? roll_controller_->compute() : 0.0f;
        double outer_servo_tilt_angle_radians = (pitch_controller_) ? pitch_controller_->compute() : 0.0f;
        double delta_motor_pwm = (yaw_controller_) ? yaw_controller_->compute() : 0.0f;

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
        publish_attitude_controller_debug();
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

    // TODO: if FlightMode::MISSION_COMPLETE => slow descent - keep algorithms running, thrust -> hover thrust * 0.9 for 1s => hover thrust
}

void BaselinePIDController::publish_attitude_controller_debug() {
    AttitudeControllerDebug msg{};

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
    attitude_controller_debug_publisher_->publish(msg);
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

void BaselinePIDController::publish_position_controller_debug() {
    PositionControllerDebug msg{};

    msg.position[0] = position_controller_->position_[0];
    msg.position[1] = position_controller_->position_[1];
    msg.position[2] = position_controller_->position_[2];
    msg.position_setpoint[0] = position_controller_->position_setpoint_[0];
    msg.position_setpoint[1] = position_controller_->position_setpoint_[1];
    msg.position_setpoint[2] = position_controller_->position_setpoint_[2];

    msg.velocity[0] = position_controller_->velocity_[0];
    msg.velocity[1] = position_controller_->velocity_[1];
    msg.velocity[2] = position_controller_->velocity_[2];
    msg.velocity_setpoint[0] = position_controller_->velocity_setpoint_[0];
    msg.velocity_setpoint[1] = position_controller_->velocity_setpoint_[1];
    msg.velocity_setpoint[2] = position_controller_->velocity_setpoint_[2];

    msg.acceleration[0] = position_controller_->acceleration_[0];
    msg.acceleration[1] = position_controller_->acceleration_[1];
    msg.acceleration[2] = position_controller_->acceleration_[2];
    msg.acceleration_setpoint[0] = position_controller_->acceleration_setpoint_[0];
    msg.acceleration_setpoint[1] = position_controller_->acceleration_setpoint_[1];
    msg.acceleration_setpoint[2] = position_controller_->acceleration_setpoint_[2];

    msg.yaw_angle_setpoint = radians_to_degrees(position_controller_->yaw_angle_setpoint_);

    msg.desired_acceleration[0] = position_controller_->desired_acceleration_[0];
    msg.desired_acceleration[1] = position_controller_->desired_acceleration_[1];
    msg.desired_acceleration[2] = position_controller_->desired_acceleration_[2];
    msg.desired_attitude[0] = radians_to_degrees(position_controller_->desired_attitude_[0]);
    msg.desired_attitude[1] = radians_to_degrees(position_controller_->desired_attitude_[1]);
    msg.desired_attitude[2] = radians_to_degrees(position_controller_->desired_attitude_[2]);
    msg.desired_thrust = position_controller_->desired_thrust_;

    msg.stamp = this->get_clock()->now();
    position_controller_debug_publisher_->publish(msg);
}

void BaselinePIDController::publish_servo_pwm(const double inner_servo_pwm, const double outer_servo_pwm)
{
    ActuatorServos msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    if (servo_active_) {
        msg.control[1] = inner_servo_pwm;
        msg.control[0] = outer_servo_pwm;
    } else {
        // If servos are not active, disable them
        msg.control[0] = 0.;
        msg.control[1] = 0.;
    }
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
    if (motor_active_) {
        msg.control[0] = upwards_motor_pwm;
        msg.control[1] = downwards_motor_pwm;
    } else {
        // If motors are not active, disable them
        msg.control[0] = NAN;
        msg.control[1] = NAN;
    }
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
