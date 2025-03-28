#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_baro.hpp>

/**
 * @brief Demo Node for SensorBaro
 */
class DemoSensorBaro : public rclcpp::Node
{
	public:
		explicit DemoSensorBaro() : Node("sensor_baro")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "sensor_baro!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::SensorBaro>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_baro node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorBaro>());

	rclcpp::shutdown();
	return 0;
}
