#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_mag.hpp>

/**
 * @brief Demo Node for SensorMag
 */
class DemoSensorMag : public rclcpp::Node
{
	public:
		explicit DemoSensorMag() : Node("sensor_mag")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "sensor_mag!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::SensorMag>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::SensorMag>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_mag node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorMag>());

	rclcpp::shutdown();
	return 0;
}
