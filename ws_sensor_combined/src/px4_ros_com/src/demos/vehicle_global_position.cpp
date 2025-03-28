#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

/**
 * @brief Demo Node for VehicleGlobalPosition
 */
class DemoVehicleGlobalPosition : public rclcpp::Node
{
	public:
		explicit DemoVehicleGlobalPosition() : Node("vehicle_global_position")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "vehicle_global_position!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_global_position node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoVehicleGlobalPosition>());

	rclcpp::shutdown();
	return 0;
}
