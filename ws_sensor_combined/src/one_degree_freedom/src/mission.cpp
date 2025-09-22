#include <rclcpp/rclcpp.hpp>

#include <one_degree_freedom/msg/trajectory_setpoint.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <one_degree_freedom/constants.hpp>

#include <one_degree_freedom/frame_transforms.h>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::frame_transforms;
using namespace one_degree_freedom::constants::setpoint;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::flight_mode;
using namespace one_degree_freedom::constants::flight_mode;

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
		flight_mode_set_publisher_{this->create_publisher<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_SET_TOPIC, qos_
		)},
		flight_mode_get_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightMode>(
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
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(
			CONTROLLER_INPUT_TRAJECTORY_SETPOINT_TOPIC, qos_
		)}
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
		mission_trajectory_setpoint_active_ = this->get_parameter(MISSION_TRAJECTORY_SETPOINT_ACTIVE_PARAM).as_bool();
		RCLCPP_INFO(this->get_logger(), "Mission trajectory setpoint active: %s", mission_trajectory_setpoint_active_ ? "true" : "false");
    }

private:
	std::atomic<uint8_t> flight_mode_;

	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	std::atomic<bool> flag_flight_mode_requested_;
    rclcpp::Publisher<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_set_publisher_;
	void request_flight_mode(uint8_t flight_mode);

    rclcpp::Subscription<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_get_subscriber_;
	void response_flight_mode_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response);

	rclcpp::TimerBase::SharedPtr flight_mode_timer_;
	void flight_mode();

	rclcpp::TimerBase::SharedPtr mission_timer_;
	void mission();

	rclcpp::Publisher<Vector3Stamped>::SharedPtr attitude_setpoint_publisher_;
	void publish_attitude_setpoint_radians(Eigen::Vector3d setpoint_radians);

	rclcpp::Publisher<Vector3Stamped>::SharedPtr translation_position_setpoint_publisher_;
	void publish_translation_position_setpoint_radians(Eigen::Vector3d translation_setpoint_meters);

	bool mission_trajectory_setpoint_active_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters
	);
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
			}
			break;	

		case FlightMode::ABORT:
			RCLCPP_ERROR(this->get_logger(), "Mission Aborted!");
			rclcpp::shutdown();
		}
}

void Mission::mission() {
    if (flight_mode_.load() == FlightMode::IN_MISSION) {
        static rclcpp::Time IN_MISSION_time = this->get_clock()->now();
        auto current_time = this->get_clock()->now();
        auto elapsed_time = current_time - IN_MISSION_time;

        // Log mission progress periodically (every second)
        static rclcpp::Time last_log_time = IN_MISSION_time;
        if (current_time - last_log_time > 10s) {
            last_log_time = current_time;
            RCLCPP_INFO(
                this->get_logger(),
                "Mission in progress - Elapsed time: %.2f seconds",
                elapsed_time.seconds()
            );
        }

		// Remove mission time limit

		// static bool first_run = true;
        // if (elapsed_time > 120s && first_run) {
		// 	first_run = false;
        //     // Mission completion
        //     RCLCPP_INFO(this->get_logger(), "Mission complete, switching to MISSION_COMPLETE mode");
        //     request_flight_mode(FlightMode::MISSION_COMPLETE);
        // }

		if (mission_trajectory_setpoint_active_) {
			#include "setpoints.h"
			static uint32_t index = 0;
			if (index >= sizeof(Setpoints) / sizeof(Setpoints[0])) {
				RCLCPP_INFO(this->get_logger(), "Mission Trajectory Tracking Complete! No more setpoints");
				// request_flight_mode(FlightMode::MISSION_COMPLETE);
				return;
			}

			auto new_setpoint = Setpoints[index];

			TrajectorySetpoint setpoint {};
			// (void) new_setpoint[0]; // time
			setpoint.position[0]     = new_setpoint[1];
			setpoint.position[1]     = new_setpoint[2];
			setpoint.position[2]     = new_setpoint[3];
			setpoint.velocity[0]     = new_setpoint[4];
			setpoint.velocity[1]     = new_setpoint[5];
			setpoint.velocity[2]     = new_setpoint[6];
			setpoint.acceleration[0] = new_setpoint[7];
			setpoint.acceleration[1] = new_setpoint[8];
			setpoint.acceleration[2] = new_setpoint[9];
			setpoint.jerk[0]         = 0.0f;
			setpoint.jerk[1]         = 0.0f;
			setpoint.jerk[2]         = 0.0f;
			setpoint.snap[0]         = 0.0f;
			setpoint.snap[1]         = 0.0f;
			setpoint.snap[2]         = 0.0f;

			trajectory_setpoint_publisher_->publish(setpoint);
			index++;
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

void Mission::publish_translation_position_setpoint_radians(Eigen::Vector3d translation_setpoint_meters)
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
            std::vector<double> translation_position_setpoint_meters = param.as_double_array();

            if (translation_position_setpoint_meters.size() != 3) {
                RCLCPP_ERROR(this->get_logger(), "Invalid setpoint size, expected 3 values.");
                result.successful = false;
                result.reason = "Invalid setpoint size";
                return result;
            }

            publish_translation_position_setpoint_radians(Eigen::Vector3d(
				translation_position_setpoint_meters[0],
				translation_position_setpoint_meters[1],
				translation_position_setpoint_meters[2]
			));

			RCLCPP_INFO(this->get_logger(), 
				"Updated position setpoint to: [%f, %f, %f]",
				translation_position_setpoint_meters[0],
				translation_position_setpoint_meters[1],
				translation_position_setpoint_meters[2]
			);
        }
		else if (param.get_name() == FLIGHT_MODE_PARAM) {
            uint8_t new_flight_mode = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated flight mode to: %d", new_flight_mode);
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
		one_degree_freedom::msg::FlightMode msg {};

		msg.flight_mode = flight_mode;
		msg.stamp = this->get_clock()->now();

		flight_mode_set_publisher_->publish(msg);
	}
}

void Mission::response_flight_mode_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response)
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
