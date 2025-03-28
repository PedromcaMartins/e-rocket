#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

/**
 * @brief Demo Node for VehicleAttitude
 */
class DemoVehicleAttitude : public rclcpp::Node
{
	public:
		explicit DemoVehicleAttitude() : Node("vehicle_attitude")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "vehicle_attitude!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_attitude node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoVehicleAttitude>());

	rclcpp::shutdown();
	return 0;
}
