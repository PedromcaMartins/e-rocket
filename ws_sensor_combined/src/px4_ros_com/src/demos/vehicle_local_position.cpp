#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

/**
 * @brief Demo Node for VehicleLocalPosition
 */
class DemoVehicleLocalPosition : public rclcpp::Node
{
	public:
		explicit DemoVehicleLocalPosition() : Node("vehicle_local_position")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "vehicle_local_position!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoVehicleLocalPosition>());

	rclcpp::shutdown();
	return 0;
}
