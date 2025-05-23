#include <rclcpp/rclcpp.hpp>

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
class Px4Ros2CommunicationTest : public rclcpp::Node
{
public: 
    Px4Ros2CommunicationTest() : 
		Node("px4_ros2_communication_test"),
		flight_mode_client_{this->create_client<one_degree_freedom::srv::FlightMode>(FLIGHT_MODE_TOPIC)}
    {
		RCLCPP_INFO(this->get_logger(), "Waiting for %s flight_mode service", "");
		while (!flight_mode_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		RCLCPP_INFO(this->get_logger(), "flight_mode service active");

		auto request = std::make_shared<one_degree_freedom::srv::FlightMode::Request>();

		std::this_thread::sleep_for(std::chrono::seconds(5));
		RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode");
		request->flight_mode = FLIGHT_MODE_OFFBOARD;
		flight_mode_client_->async_send_request(request);

		std::this_thread::sleep_for(std::chrono::seconds(5));
		RCLCPP_INFO(this->get_logger(), "Arming the vehicle");
		request->flight_mode = FLIGHT_MODE_ARM;
		flight_mode_client_->async_send_request(request);

		std::this_thread::sleep_for(std::chrono::seconds(5));
		RCLCPP_INFO(this->get_logger(), "Disarming the vehicle");
		request->flight_mode = FLIGHT_MODE_DISARM;
		flight_mode_client_->async_send_request(request);

		std::this_thread::sleep_for(std::chrono::seconds(5));
		RCLCPP_INFO(this->get_logger(), "Switching to Manual mode");
		request->flight_mode = FLIGHT_MODE_MANUAL;
		flight_mode_client_->async_send_request(request);
    }

	void switch_to_offboard_mode();
	void switch_to_manual_mode();
	void arm();
	void disarm();

private:
    rclcpp::Client<one_degree_freedom::srv::FlightMode>::SharedPtr flight_mode_client_;

	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<one_degree_freedom::srv::FlightMode>::SharedFuture future);
};

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Communication node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2CommunicationTest>());

	rclcpp::shutdown();
	return 0;
}
