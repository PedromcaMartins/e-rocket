#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/srv/vehicle_command.hpp>
#include <one_degree_freedom/srv/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::px4_ros2_communication;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2Communication : public rclcpp::Node
{
public: 
    Px4Ros2Communication() : 
		Node("px4_ros2_communication"),
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("fmu/vehicle_command")},
		flight_mode_service_{this->create_service<one_degree_freedom::srv::FlightMode>(
			FLIGHT_MODE_TOPIC, 
			std::bind(&Px4Ros2Communication::handle_flight_mode_service, this, std::placeholders::_1, std::placeholders::_2)
		)}
    {
		test_timer_ = this->create_wall_timer(
            1s, 
            [this] {
				auto request = std::make_shared<one_degree_freedom::srv::FlightMode::Request>();
				auto response = std::make_shared<one_degree_freedom::srv::FlightMode::Response>();

				switch (counter_) {
				case 5: 
					RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode");
					request->flight_mode = FLIGHT_MODE_OFFBOARD;
					handle_flight_mode_service(
						request,
						response
					);
					break;

				case 10:
					RCLCPP_INFO(this->get_logger(), "Arming the vehicle");
					request->flight_mode = FLIGHT_MODE_ARM;
					handle_flight_mode_service(
						request,
						response
					);
					break;

				case 15:
					RCLCPP_INFO(this->get_logger(), "Disarming the vehicle");
					request->flight_mode = FLIGHT_MODE_DISARM;
					handle_flight_mode_service(
						request,
						response
					);
					break;

				case 20:
					RCLCPP_INFO(this->get_logger(), "Switching to Manual mode");
					request->flight_mode = FLIGHT_MODE_MANUAL;
					handle_flight_mode_service(
						request,
						response
					);
					break;
				}
				counter_++;
			}
        );
	}

private:
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	rclcpp::Service<one_degree_freedom::srv::FlightMode>::SharedPtr flight_mode_service_;

    rclcpp::TimerBase::SharedPtr test_timer_;
	uint counter_ = 0;

	void handle_flight_mode_service(
		const std::shared_ptr<one_degree_freedom::srv::FlightMode::Request> request,
		std::shared_ptr<one_degree_freedom::srv::FlightMode::Response> response
	);

	bool adapt_flight_mode_request_to_vehicle_command(
		const std::shared_ptr<one_degree_freedom::srv::FlightMode::Request> flight_mode_request,
		std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> vehicle_command_request
	);

	void adapt_vehicle_command_to_flight_mode_response(
		const std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> vehicle_command_response,
		std::shared_ptr<one_degree_freedom::srv::FlightMode::Response> flight_mode_response
	);

	void set_vehicle_command_request(
		std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request, 
		const uint16_t command, 
		const float param1 = 0.0, 
		const float param2 = 0.0
	);
};

void Px4Ros2Communication::handle_flight_mode_service(
	const std::shared_ptr<one_degree_freedom::srv::FlightMode::Request> flight_mode_request,
	std::shared_ptr<one_degree_freedom::srv::FlightMode::Response> flight_mode_response
) {
	RCLCPP_INFO(this->get_logger(), "Received flight mode service request: '%s'", flight_mode_request->flight_mode.c_str());

	while (!vehicle_command_client_->wait_for_service(1s)) {
		RCLCPP_ERROR(this->get_logger(), "VehicleCommand service not available.");
		flight_mode_response->success = false;
		flight_mode_response->message = "VehicleCommand service unavailable.";
		return;
	}

	auto vehicle_command_request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
	if (adapt_flight_mode_request_to_vehicle_command(flight_mode_request, vehicle_command_request)) {
		RCLCPP_ERROR(this->get_logger(), "Invalid flight mode request received.");
		flight_mode_response->success = false;
		flight_mode_response->message = "Invalid flight mode request received.";
		return;
	}

	// Send the request asynchronously
	auto result_future = vehicle_command_client_->async_send_request(
		vehicle_command_request, 
		[this, flight_mode_response](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
			adapt_vehicle_command_to_flight_mode_response(future.get(), flight_mode_response);
		}
	);
}

/**
 * @return true if
 */
bool Px4Ros2Communication::adapt_flight_mode_request_to_vehicle_command(
	const std::shared_ptr<one_degree_freedom::srv::FlightMode::Request> flight_mode_request,
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

void Px4Ros2Communication::adapt_vehicle_command_to_flight_mode_response(
	const std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> vehicle_command_response,
	std::shared_ptr<one_degree_freedom::srv::FlightMode::Response> flight_mode_response
) {
	auto reply = vehicle_command_response->reply;
	switch (reply.result)
	{
	case reply.VEHICLE_CMD_RESULT_ACCEPTED:
		RCLCPP_INFO(this->get_logger(), "vehicle command accepted");
		flight_mode_response->success = true;
		flight_mode_response->message = "Vehicle command accepted.";
		break;
	case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		RCLCPP_WARN(this->get_logger(), "vehicle command temporarily rejected");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command temporarily rejected.";
		break;
	case reply.VEHICLE_CMD_RESULT_DENIED:
		RCLCPP_WARN(this->get_logger(), "vehicle command denied");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command denied.";
		break;
	case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
		RCLCPP_WARN(this->get_logger(), "vehicle command unsupported");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command unsupported.";
		break;
	case reply.VEHICLE_CMD_RESULT_FAILED:
		RCLCPP_WARN(this->get_logger(), "vehicle command failed");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command failed.";
		break;
	case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
		RCLCPP_WARN(this->get_logger(), "vehicle command in progress");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command in progress.";
		break;
	case reply.VEHICLE_CMD_RESULT_CANCELLED:
		RCLCPP_WARN(this->get_logger(), "vehicle command cancelled");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command cancelled.";
		break;
	default:
		RCLCPP_WARN(this->get_logger(), "vehicle command response unknown");
		flight_mode_response->success = false;
		flight_mode_response->message = "Vehicle command response unknown.";
		break;
	}
}

void Px4Ros2Communication::set_vehicle_command_request(
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
	rclcpp::spin(std::make_shared<Px4Ros2Communication>());

	rclcpp::shutdown();
	return 0;
}
