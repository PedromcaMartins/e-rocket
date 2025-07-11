#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::flight_mode;
using namespace one_degree_freedom::constants::px4_ros2_flight_mode;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2FlightMode : public rclcpp::Node
{
public: 
    Px4Ros2FlightMode() : 
		Node("px4_ros2_flight_mode"),
        flight_mode_current_{FlightMode::INIT},
        flight_mode_requested_{FlightMode::INIT},
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command")},
		flight_mode_set_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_SET_TOPIC, qos_, 
			std::bind(&Px4Ros2FlightMode::handle_flight_mode_set, this, std::placeholders::_1)
		)},
		flight_mode_get_publisher_{this->create_publisher<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_GET_TOPIC, qos_
		)},
		offboard_control_mode_publisher_{this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10)},
		mantain_offboard_mode_timer_{this->create_wall_timer(
			std::chrono::duration<float>(MANTAIN_OFFBOARD_MODE_TIMER_PERIOD_SECONDS),
			std::bind(&Px4Ros2FlightMode::publish_offboard_control_mode, this)
		)},
		vehicle_control_mode_publisher_{this->create_publisher<px4_msgs::msg::VehicleControlMode>("/fmu/in/vehicle_control_mode", 10)}
    {
		while (!vehicle_command_client_->wait_for_service(1s)) {
			RCLCPP_WARN(this->get_logger(), "Vehicle Command Service (PX4) is unavailable");
		}

		switch_to_manual_mode();
	}

private:
	std::atomic<uint8_t> flight_mode_current_;
	std::atomic<uint8_t> flight_mode_requested_;

	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	void switch_to_offboard_mode();
	void switch_to_manual_mode();
	void arm();
	void disarm();
	void terminate_flight();

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	void send_vehicle_command_request(
		const uint16_t command, 
		const float param1 = 0.0, 
		const float param2 = 0.0
	);
	void handle_vehicle_command_response(
		rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future
	);

	rclcpp::Subscription<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_set_subscriber_;
	void handle_flight_mode_set(
		const std::shared_ptr<one_degree_freedom::msg::FlightMode> flight_mode_set_message
	);

	rclcpp::Publisher<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_get_publisher_;
	void publish_flight_mode();

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::TimerBase::SharedPtr mantain_offboard_mode_timer_;
	void publish_offboard_control_mode();

	rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_publisher_;
	void publish_vehicle_control_mode();

};

void Px4Ros2FlightMode::switch_to_offboard_mode() {
	send_vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

void Px4Ros2FlightMode::switch_to_manual_mode() {
	send_vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
	RCLCPP_INFO(this->get_logger(), "Manual mode command sent");
}

void Px4Ros2FlightMode::arm() {
	send_vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void Px4Ros2FlightMode::disarm() {
	send_vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void Px4Ros2FlightMode::terminate_flight() {
	send_vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1);
	RCLCPP_INFO(this->get_logger(), "Terminate Flight command sent");
}

void Px4Ros2FlightMode::handle_flight_mode_set(
	const std::shared_ptr<one_degree_freedom::msg::FlightMode> flight_mode_set
) {
	if (!vehicle_command_client_->wait_for_service(1s)) {
		RCLCPP_WARN(this->get_logger(), "Vehicle Command Service (PX4) is unavailable");
		return;
	}

	auto flight_mode_current = flight_mode_current_.load();
	auto flight_mode_requested = flight_mode_set->flight_mode;
	flight_mode_requested_.store(flight_mode_requested);

	if (flight_mode_current == FlightMode::INIT && flight_mode_requested == FlightMode::PRE_ARM) {
		RCLCPP_INFO(this->get_logger(), "Received request to change flight mode to PRE_ARM");
		switch_to_offboard_mode();
	}
	else if (flight_mode_current == FlightMode::PRE_ARM && flight_mode_requested == FlightMode::ARM) {
		RCLCPP_INFO(this->get_logger(), "Received request to change flight mode to ARM");
		arm();
	}
	else if (flight_mode_current == FlightMode::ARM && flight_mode_requested == FlightMode::IN_MISSION) {
		RCLCPP_INFO(this->get_logger(), "Received request to change flight mode to IN_MISSION");

		// no need to change PX4 internal state
		publish_vehicle_control_mode();
		flight_mode_current_.store(FlightMode::IN_MISSION);
		publish_flight_mode();
	}
	else if (flight_mode_current == FlightMode::IN_MISSION && flight_mode_requested == FlightMode::MISSION_COMPLETE) {
		RCLCPP_INFO(this->get_logger(), "Received request to change flight mode to MISSION_COMPLETE");
		disarm();
	}
	else if (flight_mode_requested == FlightMode::ABORT) {
		RCLCPP_INFO(this->get_logger(), "Received request to change flight mode to ABORT");
		terminate_flight();
	}
	else if (flight_mode_current == FlightMode::ABORT) {
		rclcpp::shutdown();
	}
	else {
		RCLCPP_ERROR(this->get_logger(), "Received invalid request to change flight mode");
	}
}

void Px4Ros2FlightMode::handle_vehicle_command_response(
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future
) {
	auto vehicle_command_response = future.get();
	auto reply = vehicle_command_response->reply;

	// if request is success 
	if (reply.result == reply.VEHICLE_CMD_RESULT_ACCEPTED) {
		flight_mode_current_.store(flight_mode_requested_.load());
		RCLCPP_INFO(this->get_logger(), "Flight Mode change successful!");
	} else {
		flight_mode_requested_.store(flight_mode_current_.load());
		RCLCPP_WARN(this->get_logger(), "Flight Mode change unsuccessful! Error: %d", reply.result);
	}

	publish_flight_mode();
}

void Px4Ros2FlightMode::publish_flight_mode() {
	one_degree_freedom::msg::FlightMode msg {};

	msg.flight_mode = flight_mode_current_.load();
	msg.stamp = this->get_clock()->now();

	flight_mode_get_publisher_->publish(msg);
}

void Px4Ros2FlightMode::send_vehicle_command_request(
	const uint16_t command, 
	const float param1, 
	const float param2
) {
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	request->request = msg;

	// Send the request asynchronously
	vehicle_command_client_->async_send_request(
		request, 
		std::bind(&Px4Ros2FlightMode::handle_vehicle_command_response, this, std::placeholders::_1)
	);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only direct actuator is active.
 */
void Px4Ros2FlightMode::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the vehicle control mode.
 *        For this example, we are setting the vehicle to offboard mode.
 */
void Px4Ros2FlightMode::publish_vehicle_control_mode()
{
	px4_msgs::msg::VehicleControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.flag_armed = true;

	msg.flag_multicopter_position_control_enabled = false;

	msg.flag_control_manual_enabled = false;
	msg.flag_control_auto_enabled = false;
	msg.flag_control_offboard_enabled = true;
	msg.flag_control_position_enabled = false;
	msg.flag_control_velocity_enabled = false;
	msg.flag_control_altitude_enabled = false;
	msg.flag_control_climb_rate_enabled = false;
	msg.flag_control_acceleration_enabled = false;
	msg.flag_control_attitude_enabled = false;
	msg.flag_control_rates_enabled = false;
	msg.flag_control_allocation_enabled = false;
	msg.flag_control_termination_enabled = false;

	msg.source_id = 1;

	vehicle_control_mode_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "Vehicle control mode command sent");
}


int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Flight Mode node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2FlightMode>());

	rclcpp::shutdown();
	return 0;
}
