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
		flight_mode_set_publisher_{this->create_publisher<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_SET_TOPIC, qos_
		)},
		flight_mode_get_subscriber_{this->create_subscription<one_degree_freedom::msg::FlightMode>(
			FLIGHT_MODE_GET_TOPIC, qos_, 
			std::bind(&Px4Ros2FlightModeTest::response_callback, this, std::placeholders::_1)
		)},
		flight_mode_{FlightMode::INIT}
    {
        test_timer_ = this->create_wall_timer(5000ms, // 5 seconds
            [this]() {
				switch (flight_mode_.load()) {
				case FlightMode::INIT:
					RCLCPP_INFO(this->get_logger(), "Switching to INIT mode");
					request_flight_mode(FlightMode::PRE_ARM);
				break;

				case FlightMode::PRE_ARM:
					RCLCPP_INFO(this->get_logger(), "Switching to PRE_ARM mode");
					request_flight_mode(FlightMode::ARM);
				break;

				case FlightMode::ARM:
					RCLCPP_INFO(this->get_logger(), "Switching to ARM mode");
					request_flight_mode(FlightMode::MISSION_START);
				break;

				case FlightMode::MISSION_START:
					RCLCPP_INFO(this->get_logger(), "Switching to MISSION_START mode");
					request_flight_mode(FlightMode::MISSION_END);
				break;

				case FlightMode::MISSION_END:
					RCLCPP_INFO(this->get_logger(), "Switching to MISSION_END mode");
					request_flight_mode(FlightMode::ABORT);
				break;
				}
			}
		);
    }

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

    rclcpp::Publisher<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_set_publisher_;
    rclcpp::Subscription<one_degree_freedom::msg::FlightMode>::SharedPtr flight_mode_get_subscriber_;

	std::atomic<uint8_t> flight_mode_;

	rclcpp::TimerBase::SharedPtr test_timer_;

	void request_flight_mode(uint8_t flight_mode);
	void response_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response);
};

void Px4Ros2FlightModeTest::request_flight_mode(uint8_t flight_mode)
{
	one_degree_freedom::msg::FlightMode msg {};

	msg.flight_mode = flight_mode;
    msg.stamp = this->get_clock()->now();

	flight_mode_set_publisher_->publish(msg);
}

void Px4Ros2FlightModeTest::response_callback(std::shared_ptr<one_degree_freedom::msg::FlightMode> response)
{
	flight_mode_.store(response->flight_mode);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Flight Mode Test node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2FlightModeTest>());

	rclcpp::shutdown();
	return 0;
}
