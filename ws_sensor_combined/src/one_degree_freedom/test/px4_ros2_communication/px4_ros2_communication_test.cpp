#include <rclcpp/rclcpp.hpp>

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
class Px4Ros2FlightModeTest : public rclcpp::Node
{
public: 
    Px4Ros2FlightModeTest() : 
		Node("px4_ros2_flight_mode_test"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
		flight_mode_client_request_publisher_{this->create_publisher<one_degree_freedom::msg::FlightModeRequest>(
			FLIGHT_MODE_REQUEST_TOPIC, qos_
		)},
		flight_mode_client_response_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightModeResponse>(
			FLIGHT_MODE_RESPONSE_TOPIC, qos_, 
			std::bind(&Px4Ros2FlightModeTest::response_callback, this, std::placeholders::_1)
		)},
		flight_mode_response_received_{false},
		flight_mode_response_success_{false}
    {
        test_timer_ = this->create_wall_timer(5000ms, // 5 seconds
            [this]() {
				if (flight_mode_response_received_.load()) {
					flight_mode_response_received_.store(false);
					auto success = flight_mode_response_success_.load();
					RCLCPP_INFO(this->get_logger(), "Flight mode response received: %s", success ? "Success" : "Failure");

					if (success) {
						// change to next state
						counter_++;
					}
				} else {
					RCLCPP_WARN(this->get_logger(), "No flight mode response received yet.");
				}

				switch (counter_ % 4) {
				case 0:
					RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode");
					request_flight_mode(FLIGHT_MODE_OFFBOARD);
				break;

				case 1:
					RCLCPP_INFO(this->get_logger(), "Arming the vehicle");
					request_flight_mode(FLIGHT_MODE_ARM);
				break;

				case 2:
					RCLCPP_INFO(this->get_logger(), "Disarming the vehicle");
					request_flight_mode(FLIGHT_MODE_DISARM);
				break;

				case 3:
					RCLCPP_INFO(this->get_logger(), "Switching to Manual mode");
					request_flight_mode(FLIGHT_MODE_MANUAL);
				break;
				}
			}
		);
    }

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

    rclcpp::Publisher<one_degree_freedom::msg::FlightModeRequest>::SharedPtr flight_mode_client_request_publisher_;
    rclcpp::Subscription<one_degree_freedom::msg::FlightModeResponse>::SharedPtr flight_mode_client_response_subscriber_;

	std::atomic<bool> flight_mode_response_received_;
	std::atomic<bool> flight_mode_response_success_;

	rclcpp::TimerBase::SharedPtr test_timer_;
	uint counter_ = 0;

	void request_flight_mode(const char * flight_mode);
	void response_callback(std::shared_ptr<one_degree_freedom::msg::FlightModeResponse> response);
};

void Px4Ros2FlightModeTest::request_flight_mode(const char * flight_mode)
{
	one_degree_freedom::msg::FlightModeRequest msg {};
	msg.flight_mode = std::string(flight_mode);
    msg.stamp = this->get_clock()->now();

	flight_mode_client_request_publisher_->publish(msg);
}

void Px4Ros2FlightModeTest::response_callback(std::shared_ptr<one_degree_freedom::msg::FlightModeResponse> response)
{
	RCLCPP_INFO(this->get_logger(), "Received flight mode response: '%s'", response->message.c_str());
	flight_mode_response_success_.store(response->success);
	flight_mode_response_received_.store(true);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Communication node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2FlightModeTest>());

	rclcpp::shutdown();
	return 0;
}
