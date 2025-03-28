#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

/**
 * @brief Demo Node for SensorGps
 */
class DemoSensorGps : public rclcpp::Node
{
	public:
		explicit DemoSensorGps() : Node("sensor_gps")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "sensor_gps!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_gps node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorGps>());

	rclcpp::shutdown();
	return 0;
}
