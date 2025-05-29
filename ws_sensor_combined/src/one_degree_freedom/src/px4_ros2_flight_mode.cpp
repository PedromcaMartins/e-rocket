#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/srv/vehicle_command.hpp>
#include <one_degree_freedom/msg/flight_mode_request.hpp>
#include <one_degree_freedom/msg/flight_mode_response.hpp>
#include <one_degree_freedom/constants.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::px4_ros2_flight_mode;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2FlightMode : public rclcpp::Node
{
public: 
    Px4Ros2FlightMode() : 
		Node("px4_ros2_flight_mode"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("fmu/vehicle_command", qos_profile_)},
		flight_mode_request_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightModeRequest>(
			FLIGHT_MODE_REQUEST_TOPIC, qos_, 
			std::bind(&Px4Ros2FlightMode::handle_flight_mode_request, this, std::placeholders::_1)
		)},
		flight_mode_response_publisher_{this->create_publisher<one_degree_freedom::msg::FlightModeResponse>(
			FLIGHT_MODE_RESPONSE_TOPIC, qos_
		)}
    {
	}

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	rclcpp::Subscription<one_degree_freedom::msg::FlightModeRequest>::SharedPtr flight_mode_request_subscriber_;
	rclcpp::Publisher<one_degree_freedom::msg::FlightModeResponse>::SharedPtr flight_mode_response_publisher_;

	void handle_flight_mode_request(
		const std::shared_ptr<one_degree_freedom::msg::FlightModeRequest> request
	);

	void publish_flight_mode_response(
		const bool success, 
		const char * message
	);

	bool adapt_flight_mode_request_to_vehicle_command(
		const std::shared_ptr<one_degree_freedom::msg::FlightModeRequest> flight_mode_request,
		std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> vehicle_command_request
	);

	void publish_vehicle_command_as_flight_mode_response(
		const std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> vehicle_command_response
	);

	void set_vehicle_command_request(
		std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request, 
		const uint16_t command, 
		const float param1 = 0.0, 
		const float param2 = 0.0
	);
};

void Px4Ros2FlightMode::handle_flight_mode_request(
	const std::shared_ptr<one_degree_freedom::msg::FlightModeRequest> flight_mode_request
) {
	RCLCPP_INFO(this->get_logger(), "Received flight mode service request: '%s'", flight_mode_request->flight_mode.c_str());

	while (!vehicle_command_client_->wait_for_service(1s)) {
		publish_flight_mode_response(
			false,
			"VehicleCommand service unavailable."
		);
		return;
	}

	auto vehicle_command_request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
	if (adapt_flight_mode_request_to_vehicle_command(flight_mode_request, vehicle_command_request)) {
		publish_flight_mode_response(
			false, 
			"Invalid flight mode request received."
		);
		return;
	}

	// Send the request asynchronously
	auto result_future = vehicle_command_client_->async_send_request(
		vehicle_command_request, 
		[this](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
			publish_vehicle_command_as_flight_mode_response(future.get());
		}
	);
}

void Px4Ros2FlightMode::publish_flight_mode_response(const bool success, const char * message) {
	one_degree_freedom::msg::FlightModeResponse msg {};
	msg.success = success;
	msg.message = std::string(message);

	flight_mode_response_publisher_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "Published flight mode response: success=%s, message='%s'", 
		success ? "true" : "false", message);
}

/**
 * @return true if
 */
bool Px4Ros2FlightMode::adapt_flight_mode_request_to_vehicle_command(
	const std::shared_ptr<one_degree_freedom::msg::FlightModeRequest> flight_mode_request,
	std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> vehicle_command_request
) {
	auto flight_mode = flight_mode_request->flight_mode.c_str();

	if (strcmp(flight_mode, FLIGHT_MODE_OFFBOARD) == 0) {
		RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
		set_vehicle_command_request(vehicle_command_request, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	} 
	else if (strcmp(flight_mode, FLIGHT_MODE_MANUAL) == 0) {
		RCLCPP_INFO(this->get_logger(), "requesting switch to Manual mode");
		set_vehicle_command_request(vehicle_command_request, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
	} 
	else if (strcmp(flight_mode, FLIGHT_MODE_ARM) == 0) {
		RCLCPP_INFO(this->get_logger(), "requesting arm");
		set_vehicle_command_request(vehicle_command_request, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	} 
	else if (strcmp(flight_mode, FLIGHT_MODE_DISARM) == 0) {
		RCLCPP_INFO(this->get_logger(), "requesting disarm");
		set_vehicle_command_request(vehicle_command_request, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	} 
	else {
		RCLCPP_ERROR(this->get_logger(), "unknown flight mode requested");
		return true;
	}

	return false;
}

void Px4Ros2FlightMode::publish_vehicle_command_as_flight_mode_response(
	const std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> vehicle_command_response
) {
	auto reply = vehicle_command_response->reply;
	switch (reply.result)
	{
	case reply.VEHICLE_CMD_RESULT_ACCEPTED:
		RCLCPP_INFO(this->get_logger(), "vehicle command accepted");
		publish_flight_mode_response(
			true,
			"Vehicle command accepted."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		RCLCPP_WARN(this->get_logger(), "vehicle command temporarily rejected");
		publish_flight_mode_response(
			false,
			"Vehicle command temporarily rejected."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_DENIED:
		RCLCPP_WARN(this->get_logger(), "vehicle command denied");
		publish_flight_mode_response(
			false,
			"Vehicle command denied."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
		RCLCPP_WARN(this->get_logger(), "vehicle command unsupported");
		publish_flight_mode_response(
			false,
			"Vehicle command unsupported."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_FAILED:
		RCLCPP_WARN(this->get_logger(), "vehicle command failed");
		publish_flight_mode_response(
			false,
			"Vehicle command failed."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
		RCLCPP_WARN(this->get_logger(), "vehicle command in progress");
		publish_flight_mode_response(
			false,
			"Vehicle command in progress."
		);
		break;
	case reply.VEHICLE_CMD_RESULT_CANCELLED:
		RCLCPP_WARN(this->get_logger(), "vehicle command cancelled");
		publish_flight_mode_response(
			false,
			"Vehicle command cancelled."
		);
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "vehicle command response unknown");
		publish_flight_mode_response(
			false,
			"Vehicle command response unknown."
		);
		break;
	}
}

void Px4Ros2FlightMode::set_vehicle_command_request(
	std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request, 
	const uint16_t command, 
	const float param1, 
	const float param2
) {
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
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Communication node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2FlightMode>());

	rclcpp::shutdown();
	return 0;
}
