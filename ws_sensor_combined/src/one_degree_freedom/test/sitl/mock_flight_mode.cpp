#include <rclcpp/rclcpp.hpp>

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

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class MockFlightMode : public rclcpp::Node
{
public: 
    MockFlightMode() : 
		Node("mock_flight_mode"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
		flight_mode_set_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_SET_TOPIC, qos_, 
			std::bind(&MockFlightMode::handle_flight_mode_set, this, std::placeholders::_1)
		)},
		flight_mode_get_publisher_{this->create_publisher<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_GET_TOPIC, qos_
		)}
    {}

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	rclcpp::Subscription<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_set_subscriber_;
	rclcpp::Publisher<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_get_publisher_;
	void handle_flight_mode_set(
		const std::shared_ptr<one_degree_freedom::msg::FlightMode> flight_mode_set_message
	);
};

void MockFlightMode::handle_flight_mode_set(
	const std::shared_ptr<one_degree_freedom::msg::FlightMode> flight_mode_set
) {
	one_degree_freedom::msg::FlightMode msg {};

	msg.flight_mode = flight_mode_set->flight_mode;
	msg.stamp = this->get_clock()->now();

	flight_mode_get_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting Mock Flight Mode node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MockFlightMode>());

	rclcpp::shutdown();
	return 0;
}
