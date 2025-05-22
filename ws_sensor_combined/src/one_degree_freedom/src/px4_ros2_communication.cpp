#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/srv/vehicle_command.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2Communication : public rclcpp::Node
{
public: 
    Px4Ros2Communication() : 
		Node("px4_ros2_communication"),
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("fmu/vehicle_command")}
    {
		RCLCPP_INFO(this->get_logger(), "Waiting for %s vehicle_command service", "");
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		RCLCPP_INFO(this->get_logger(), "Vehicle command service active");

		test_timer_ = this->create_wall_timer(
            1s, 
            [this] {
				switch (counter_) {
				case 5: 
					RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode");
					switch_to_offboard_mode();
					break;

				case 10:
					RCLCPP_INFO(this->get_logger(), "Arming the vehicle");
					arm();
					break;

				case 15:
					RCLCPP_INFO(this->get_logger(), "Disarming the vehicle");
					disarm();
					break;

				case 20:
					RCLCPP_INFO(this->get_logger(), "Switching to Manual mode");
					switch_to_manual_mode();
					break;
				}
				counter_++;
			}
        );
    }

	void switch_to_offboard_mode();
	void switch_to_manual_mode();
	void arm();
	void disarm();

private:
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp::TimerBase::SharedPtr test_timer_;
	uint counter_ = 0;


	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
};

/**
 * @brief Send a command to switch to offboard mode
 */
void Px4Ros2Communication::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to switch to offboard mode
 */
void Px4Ros2Communication::switch_to_manual_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Manual mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void Px4Ros2Communication::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Px4Ros2Communication::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Px4Ros2Communication::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
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

	auto result = vehicle_command_client_->async_send_request(
		request, std::bind(&Px4Ros2Communication::response_callback, this, std::placeholders::_1)
	);
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void Px4Ros2Communication::response_callback(
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future
) {
	auto status = future.wait_for(1s);
	if (status == std::future_status::ready) {
		auto reply = future.get()->reply;
		switch (reply.result)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
	} else {
		RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
	}
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
