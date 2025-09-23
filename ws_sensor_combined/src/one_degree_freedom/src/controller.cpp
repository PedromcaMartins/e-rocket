#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <one_degree_freedom/msg/attitude_controller_debug.hpp>
#include <one_degree_freedom/msg/position_controller_debug.hpp>
#include <one_degree_freedom/msg/allocator_debug.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>

#include <one_degree_freedom/vehicle_constants.hpp>
#include <one_degree_freedom/controller/allocator.hpp>
#include <one_degree_freedom/controller/state.hpp>
#include <one_degree_freedom/controller/setpoint.hpp>
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

    vehicle_constants_{std::make_shared<VehicleConstants>(this)},
    state_aggregator_{std::make_unique<StateAggregator>(this, qos_)},
    setpoint_aggregator_{std::make_unique<SetpointAggregator>(this, qos_)},
    attitude_controller_{this, qos_, state_aggregator_, setpoint_aggregator_},
    position_controller_{this, qos_, vehicle_constants_, state_aggregator_, setpoint_aggregator_},
    allocator_{std::make_unique<Allocator>(vehicle_constants_)},

    servo_tilt_angle_publisher_{this->create_publisher<ActuatorServos>(
        CONTROLLER_OUTPUT_SERVO_PWM_TOPIC, qos_
    )},
    motor_thrust_publisher_{this->create_publisher<ActuatorMotors>(
        CONTROLLER_OUTPUT_MOTOR_PWM_TOPIC, qos_
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
        if (position_controller_.is_controller_active() && !attitude_controller_.are_all_controllers_active()) {
            RCLCPP_ERROR(this->get_logger(), "Position controller requires all attitude controllers to be active.");
            throw std::runtime_error("Position controller requires all attitude controllers to be active.");
        }

        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.02), 
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

    std::shared_ptr<VehicleConstants> vehicle_constants_;
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    AttitudePIDController attitude_controller_;
    PositionPIDController position_controller_;
    std::unique_ptr<Allocator> allocator_;

    //!< Time variables
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Publishers and Subscribers
	rclcpp::Publisher<ActuatorServos>::SharedPtr    servo_tilt_angle_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr    motor_thrust_publisher_;

    rclcpp::Publisher<AllocatorDebug>::SharedPtr  allocator_debug_publisher_;

	//!< Auxiliary functions
    void controller_callback();
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
            static bool first_time = false;
            if (!first_time) {
                first_time = true;
                attitude_controller_.set_current_yaw_as_origin();
            }
            // Needed to set upright
            double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(0.4f);
            auto attitude_output = attitude_controller_.compute();

            // Allocate motor thrust based on the computed torque
            Allocator::MotorAllocatorOutput motor_output = allocator_->compute_motor_allocation(
                attitude_output.delta_motor_pwm, 
                average_motor_thrust_newtons
            );

            // Publish the motor thrust
            publish_motor_pwm(
                motor_output.upwards_motor_pwm, 
                motor_output.downwards_motor_pwm
            );
        }
    }

    // only run controller when in mission
    else if (flight_mode_ == FlightMode::IN_MISSION) {
        double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(vehicle_constants_->default_motor_pwm_);

        if (position_controller_.is_controller_active()) {
            static bool mission_start = false;
            if (!mission_start) {
                mission_start = true;
                attitude_controller_.set_current_yaw_as_origin();
                position_controller_.set_current_position_as_origin();
            }

            auto position_output = position_controller_.compute();
            setpoint_aggregator_->set_attitude_setpoint({position_output.desired_attitude});
            average_motor_thrust_newtons = position_output.desired_thrust;
        }

        auto attitude_output = attitude_controller_.compute();

        Allocator::ServoAllocatorOutput servo_output = allocator_->compute_servo_allocation(
            attitude_output.inner_servo_tilt_angle,
            attitude_output.outer_servo_tilt_angle
        );

        publish_servo_pwm(
            servo_output.inner_servo_pwm,
            servo_output.outer_servo_pwm
        );

        // Allocate motor thrust based on the computed torque
        Allocator::MotorAllocatorOutput motor_output = allocator_->compute_motor_allocation(
            attitude_output.delta_motor_pwm, 
            average_motor_thrust_newtons
        );

        // Publish the motor thrust
        publish_motor_pwm(
            motor_output.upwards_motor_pwm, 
            motor_output.downwards_motor_pwm
        );

        // Publish the controller debug information
        publish_allocator_debug(
            attitude_output.inner_servo_tilt_angle,
            attitude_output.outer_servo_tilt_angle,
            servo_output.inner_servo_pwm,
            servo_output.outer_servo_pwm,

            attitude_output.delta_motor_pwm,
            average_motor_thrust_newtons,
            motor_output.upwards_motor_pwm,
            motor_output.downwards_motor_pwm
        );
    }

    // TODO: if FlightMode::MISSION_COMPLETE => slow descent - keep algorithms running, thrust -> hover thrust * 0.9 for 1s => hover thrust
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
    if (vehicle_constants_->servo_active_) {
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
    if (vehicle_constants_->motor_active_) {
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
