#include <rclcpp/rclcpp.hpp>

#include <erocket/msg/setpoint_c5.hpp>
#include <erocket/msg/flight_mode.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <erocket/constants.hpp>

#include <erocket/controller/state.hpp>
#include <erocket/mission/take_off_and_landing.hpp>
#include <erocket/emergency_switch.h>

#include <erocket/frame_transforms.h>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace erocket::msg;
using namespace erocket::frame_transforms;
using namespace erocket::constants::setpoint;
using namespace erocket::constants::controller;
using namespace erocket::constants::flight_mode;
using namespace erocket::constants::takeoff_landing;
using namespace erocket::constants::emergency;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Mission : public rclcpp::Node
{
public: 
    Mission() : 
		Node("mission"),
		flight_mode_{FlightMode::INIT},
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
    	state_aggregator_{std::make_unique<StateAggregator>(this, qos_)},
		flight_mode_set_publisher_{this->create_publisher<erocket::msg::FlightMode>(
			FLIGHT_MODE_SET_TOPIC, qos_
		)},
		flight_mode_get_subscriber_{this->create_subscription<erocket::msg::FlightMode>(
			FLIGHT_MODE_GET_TOPIC, qos_, 
			std::bind(&Mission::response_flight_mode_callback, this, std::placeholders::_1)
		)},
		mission_timer_{this->create_wall_timer(
			10ms, 
			std::bind(&Mission::mission, this)
		)},
        attitude_setpoint_publisher_{this->create_publisher<Vector3Stamped>(
            CONTROLLER_INPUT_ATTITUDE_SETPOINT_TOPIC, qos_
        )},
        translation_position_setpoint_publisher_{this->create_publisher<Vector3Stamped>(
            CONTROLLER_INPUT_TRANSLATION_POSITION_SETPOINT_TOPIC, qos_
        )},
		trajectory_setpoint_publisher_{this->create_publisher<SetpointC5>(
			CONTROLLER_INPUT_SETPOINT_C5_TOPIC, qos_
		)},
		emergency_switch_{this, qos_}
    {
        flight_mode_timer_ = this->create_wall_timer(
			1s,
            std::bind(&Mission::flight_mode, this)
		);

        // Declare the setpoint attitude parameter as array of 3 floats [roll, pitch, yaw]
        this->declare_parameter<std::vector<double>>(
            MISSION_ATTITUDE_SETPOINT_PARAM, 
            std::vector<double>{0.0, 0.0, 0.0}
        );
        this->declare_parameter<std::vector<double>>(
            MISSION_TRANSLATION_POSITION_SETPOINT_PARAM, 
            std::vector<double>{0.0, 0.0, 0.0}
        );
        this->declare_parameter<uint8_t>(FLIGHT_MODE_PARAM, FlightMode::INIT);

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Mission::parameter_callback, this, std::placeholders::_1)
        );

		this->declare_parameter<bool>(MISSION_TRAJECTORY_SETPOINT_ACTIVE_PARAM);
		trajectory_setpoint_active_ = this->get_parameter(MISSION_TRAJECTORY_SETPOINT_ACTIVE_PARAM).as_bool();
		RCLCPP_INFO(this->get_logger(), "Mission trajectory setpoint active: %s", trajectory_setpoint_active_ ? "true" : "false");

		this->declare_parameter<double>(MISSION_TAKEOFF_CLIMB_HEIGHT_PARAM);
		desired_climb_height_ = this->get_parameter(MISSION_TAKEOFF_CLIMB_HEIGHT_PARAM).as_double();

		this->declare_parameter<double>(MISSION_TAKEOFF_CLIMB_DURATION_PARAM);
		takeoff_climb_duration_ = this->get_parameter(MISSION_TAKEOFF_CLIMB_DURATION_PARAM).as_double();

		this->declare_parameter<double>(MISSION_LANDING_DESCENT_DURATION_PARAM);
		landing_descent_duration_ = this->get_parameter(MISSION_LANDING_DESCENT_DURATION_PARAM).as_double();

		RCLCPP_INFO(this->get_logger(), "Mission desired climb height: %f m, takeoff climb duration: %f s, landing descent duration: %f s",
		desired_climb_height_, takeoff_climb_duration_, landing_descent_duration_
		);
    }

private:
	std::atomic<uint8_t> flight_mode_;

	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	double desired_climb_height_;
	double takeoff_climb_duration_;
	double landing_descent_duration_;
	State ground_state_;

    std::shared_ptr<StateAggregator> state_aggregator_;

	std::atomic<bool> flag_flight_mode_requested_;
    rclcpp::Publisher<erocket::msg::FlightMode>::SharedPtr flight_mode_set_publisher_;
	void request_flight_mode(uint8_t flight_mode);

    rclcpp::Subscription<erocket::msg::FlightMode>::SharedPtr flight_mode_get_subscriber_;
	void response_flight_mode_callback(std::shared_ptr<erocket::msg::FlightMode> response);

	rclcpp::TimerBase::SharedPtr flight_mode_timer_;
	void flight_mode();

	rclcpp::TimerBase::SharedPtr mission_timer_;
	void mission();

	rclcpp::Publisher<Vector3Stamped>::SharedPtr attitude_setpoint_publisher_;
	void publish_attitude_setpoint_radians(Eigen::Vector3d setpoint_radians);

	rclcpp::Publisher<Vector3Stamped>::SharedPtr translation_position_setpoint_publisher_;
	void publish_translation_position_setpoint(Eigen::Vector3d translation_setpoint_meters);

	bool trajectory_setpoint_active_;
	rclcpp::Publisher<SetpointC5>::SharedPtr trajectory_setpoint_publisher_;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters
	);

	EmergencySwitch emergency_switch_;
};

void Mission::flight_mode() {
	switch (flight_mode_.load()) {
		case FlightMode::INIT:
			RCLCPP_INFO(this->get_logger(), "Switching to PRE_ARM mode");
			request_flight_mode(FlightMode::PRE_ARM);
			break;

		case FlightMode::PRE_ARM:
			RCLCPP_INFO(this->get_logger(), "Switching to ARM mode");
			request_flight_mode(FlightMode::ARM);
			break;

		case FlightMode::ARM:
			{
				// Changes manual mode to IN_MISSION manually

				// static rclcpp::Time t0 = this->get_clock()->now();
				// auto now = this->get_clock()->now();
				// if (now - t0 > 1s) {
				// 	RCLCPP_INFO(this->get_logger(), "Switching to IN_MISSION mode");
				// 	request_flight_mode(FlightMode::IN_MISSION);
				// }

				// sets position setpoint to current position
				static bool initialized_position_setpoint = false;
				if (!initialized_position_setpoint) {
					initialized_position_setpoint = true;
					auto state = state_aggregator_->get_state();

					SetpointC5 setpoint {};
					Eigen::Map<Eigen::Vector3d>(setpoint.position.data()) = state.position;
					setpoint.yaw = state.euler_angles[2]; // keep current yaw
					trajectory_setpoint_publisher_->publish(setpoint);
					RCLCPP_INFO(this->get_logger(), "Initialized position setpoint to current position: [%f, %f, %f], yaw: %f",
						setpoint.position[0], setpoint.position[1], setpoint.position[2], radians_to_degrees(setpoint.yaw)
					);
				}

				ground_state_ = state_aggregator_->get_state();
			}
			break;
		
		case FlightMode::IN_MISSION:
			{
				static rclcpp::Time IN_MISSION_time = this->get_clock()->now();
				auto current_time = this->get_clock()->now();
				auto elapsed_time = current_time - IN_MISSION_time;

				// Log mission progress periodically (every second)
				static rclcpp::Time last_log_time = IN_MISSION_time;
				if (current_time - last_log_time >= 10s) {
					last_log_time = current_time;
					RCLCPP_INFO(
						this->get_logger(),
						"Mission in progress - Elapsed time: %.2f seconds",
						elapsed_time.seconds()
					);
				}
			}
			break;

		case FlightMode::ABORT:
			RCLCPP_ERROR(this->get_logger(), "Mission Aborted!");
			rclcpp::shutdown();
			break;
		}
}

void Mission::mission() {
	if (emergency_switch_.emergency_switch_on()) {
		if (flight_mode_.load() != FlightMode::ABORT) {
			RCLCPP_ERROR(this->get_logger(), "Emergency switch activated! Aborting mission...");
			request_flight_mode(FlightMode::ABORT);
		}
		return;
	}

	if (flight_mode_.load() == FlightMode::TAKE_OFF) {
		static rclcpp::Time takeoff_start_time = this->get_clock()->now();

		auto current_time = this->get_clock()->now();
		double t = (current_time - takeoff_start_time).seconds(); 
		auto new_setpoint = compute_takeoff(
			t,
			ground_state_.position,
			desired_climb_height_,
			takeoff_climb_duration_
		);

		SetpointC5 setpoint {};
		// (void) new_setpoint[0]; // time
		Eigen::Map<Eigen::Vector3d>(setpoint.position.data())     = new_setpoint.pd;
		Eigen::Map<Eigen::Vector3d>(setpoint.velocity.data())     = new_setpoint.pd_dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.acceleration.data()) = new_setpoint.pd_2dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.jerk.data()) = new_setpoint.pd_3dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.snap.data()) = new_setpoint.pd_4dot;
		setpoint.yaw = std::nan("");

		trajectory_setpoint_publisher_->publish(setpoint);
	}

    if (flight_mode_.load() == FlightMode::IN_MISSION) {
		if (trajectory_setpoint_active_) {
			#include "setpoints.h"
			static uint32_t index = 0;
			if (index >= sizeof(Setpoints) / sizeof(Setpoints[0])) {
				RCLCPP_INFO(this->get_logger(), "Mission Trajectory Tracking Complete! No more setpoints");
				RCLCPP_INFO(this->get_logger(), "Switching to LANDING mode");
				request_flight_mode(FlightMode::LANDING);
				return;
			}

			auto new_setpoint = Setpoints[index];

			SetpointC5 setpoint {};
			// (void) new_setpoint[0]; // time
			setpoint.position[0]     = new_setpoint[1];
			setpoint.velocity[0]     = new_setpoint[2];
			setpoint.acceleration[0] = new_setpoint[3];
			setpoint.position[1]     = new_setpoint[4];
			setpoint.velocity[1]     = new_setpoint[5];
			setpoint.acceleration[1] = new_setpoint[6];
			setpoint.position[2]     = new_setpoint[7];
			setpoint.velocity[2]     = new_setpoint[8];
			setpoint.acceleration[2] = new_setpoint[9];
			setpoint.yaw             = std::nan("");

			trajectory_setpoint_publisher_->publish(setpoint);
			index++;
		}
    }

	if (flight_mode_.load() == FlightMode::LANDING) {
		static State initial_landing_state = state_aggregator_->get_state();

		auto current_time = this->get_clock()->now();
		static rclcpp::Time landing_start_time = this->get_clock()->now();
		double t = (current_time - landing_start_time).seconds(); 
		auto new_setpoint = compute_landing(
			t,
			initial_landing_state.position,
			ground_state_.position,
			landing_descent_duration_
		);

		SetpointC5 setpoint {};
		// (void) new_setpoint[0]; // time
		Eigen::Map<Eigen::Vector3d>(setpoint.position.data())     = new_setpoint.pd;
		Eigen::Map<Eigen::Vector3d>(setpoint.velocity.data())     = new_setpoint.pd_dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.acceleration.data()) = new_setpoint.pd_2dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.jerk.data()) = new_setpoint.pd_3dot;
		Eigen::Map<Eigen::Vector3d>(setpoint.snap.data()) = new_setpoint.pd_4dot;
		setpoint.yaw = std::nan("");

		trajectory_setpoint_publisher_->publish(setpoint);

		if (t >= landing_descent_duration_) { // landing duration
			RCLCPP_INFO(this->get_logger(), "Landing sequence completed. Mission complete.");
			RCLCPP_INFO(this->get_logger(), "Switching to MISSION_COMPLETE mode");
			request_flight_mode(FlightMode::MISSION_COMPLETE);
			rclcpp::sleep_for(100ms);
		}
	}
}

void Mission::publish_attitude_setpoint_radians(Eigen::Vector3d setpoint_radians)
{
    Vector3Stamped msg{};
    msg.header.stamp = this->get_clock()->now();
    msg.vector.x = setpoint_radians[0];
    msg.vector.y = setpoint_radians[1];
    msg.vector.z = setpoint_radians[2];
    attitude_setpoint_publisher_->publish(msg);
}

void Mission::publish_translation_position_setpoint(Eigen::Vector3d translation_setpoint_meters)
{
    Vector3Stamped msg{};
    msg.header.stamp = this->get_clock()->now();
    msg.vector.x = translation_setpoint_meters[0];
    msg.vector.y = translation_setpoint_meters[1];
    msg.vector.z = translation_setpoint_meters[2];
    translation_position_setpoint_publisher_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult Mission::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == MISSION_ATTITUDE_SETPOINT_PARAM) {
			if (trajectory_setpoint_active_) {
				RCLCPP_ERROR(this->get_logger(), "Ignoring attitude setpoint change while trajectory setpoint is active.");
                result.successful = false;
				result.reason = "Trajectory setpoint active";
                return result;
			}
			if (flight_mode_ != FlightMode::IN_MISSION) {
				RCLCPP_ERROR(this->get_logger(), "Ignoring attitude setpoint change while not in mission.");
                result.successful = false;
				result.reason = "Not in mission";
                return result;
			}

            std::vector<double> attitude_setpoint_degrees = param.as_double_array();

            if (attitude_setpoint_degrees.size() != 3) {
                RCLCPP_ERROR(this->get_logger(), "Invalid attitude setpoint size, expected 3 values.");
                result.successful = false;
                result.reason = "Invalid attitude setpoint size";
                return result;
            }

			Eigen::Vector3d attitude_setpoint_radians = degrees_to_radians(Eigen::Vector3d(
				attitude_setpoint_degrees[0],
				attitude_setpoint_degrees[1],
				attitude_setpoint_degrees[2]
			));

            publish_attitude_setpoint_radians(attitude_setpoint_radians);

			RCLCPP_INFO(this->get_logger(), 
				"Updated attitude setpoint to: [%f, %f, %f]",
				attitude_setpoint_degrees[0],
				attitude_setpoint_degrees[1],
				attitude_setpoint_degrees[2]
			);
        } 
		else if (param.get_name() == MISSION_TRANSLATION_POSITION_SETPOINT_PARAM) {
			if (trajectory_setpoint_active_) {
				RCLCPP_ERROR(this->get_logger(), "Ignoring translation position setpoint change while trajectory setpoint is active.");
                result.successful = false;
				result.reason = "Trajectory setpoint active";
                return result;
			}
			if (flight_mode_ != FlightMode::IN_MISSION) {
				RCLCPP_ERROR(this->get_logger(), "Ignoring attitude setpoint change while not in mission.");
                result.successful = false;
				result.reason = "Not in mission";
                return result;
			}

            std::vector<double> translation_position_setpoint_meters = param.as_double_array();

            if (translation_position_setpoint_meters.size() != 3) {
                RCLCPP_ERROR(this->get_logger(), "Invalid setpoint size, expected 3 values.");
                result.successful = false;
                result.reason = "Invalid setpoint size";
                return result;
            }

            publish_translation_position_setpoint(
				Eigen::Map<Eigen::Vector3d>(translation_position_setpoint_meters.data())
			);

			RCLCPP_INFO(this->get_logger(), 
				"Updated position setpoint by: [%f, %f, %f]",
				translation_position_setpoint_meters[0],
				translation_position_setpoint_meters[1],
				translation_position_setpoint_meters[2]
			);
        }
		else if (param.get_name() == FLIGHT_MODE_PARAM) {
            uint8_t new_flight_mode = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Requesting flight mode to: %d", new_flight_mode);
			request_flight_mode(new_flight_mode);

			while (new_flight_mode == FlightMode::ABORT) {
				request_flight_mode(FlightMode::ABORT);
			}
        }
    }

    return result;
}

void Mission::request_flight_mode(uint8_t flight_mode)
{
	if (flight_mode == FlightMode::ABORT || flag_flight_mode_requested_.load() == false) {
		flag_flight_mode_requested_.store(true);
		erocket::msg::FlightMode msg {};

		msg.flight_mode = flight_mode;
		msg.stamp = this->get_clock()->now();

		flight_mode_set_publisher_->publish(msg);
	}
}

void Mission::response_flight_mode_callback(std::shared_ptr<erocket::msg::FlightMode> response)
{
	RCLCPP_INFO(this->get_logger(), "Flight Mode switch %s", (response->flight_mode != flight_mode_.load()) ? "Confirmed" : "Failed");
	flight_mode_.store(response->flight_mode);

	// reset flag
	flag_flight_mode_requested_.store(false);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting Mission node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mission>());

	rclcpp::shutdown();
	return 0;
}
