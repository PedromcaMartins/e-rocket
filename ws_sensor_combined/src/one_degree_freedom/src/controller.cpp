#include <rclcpp/rclcpp.hpp>
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
    allocator_{std::make_unique<Allocator>(this, qos_, vehicle_constants_)},

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

	//!< Auxiliary functions
    void controller_callback();

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

        allocator_->compute_servo_allocation({
            inner_servo_tilt_angle_radians,
            outer_servo_tilt_angle_radians
        });

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
            allocator_->compute_motor_allocation({
                attitude_output.delta_motor_pwm, 
                average_motor_thrust_newtons
            });
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

        allocator_->compute_servo_allocation({
            attitude_output.inner_servo_tilt_angle,
            attitude_output.outer_servo_tilt_angle
        });

        // Allocate motor thrust based on the computed torque
        allocator_->compute_motor_allocation({
            attitude_output.delta_motor_pwm, 
            average_motor_thrust_newtons
        });
    }

    // TODO: if FlightMode::MISSION_COMPLETE => slow descent - keep algorithms running, thrust -> hover thrust * 0.9 for 1s => hover thrust
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
