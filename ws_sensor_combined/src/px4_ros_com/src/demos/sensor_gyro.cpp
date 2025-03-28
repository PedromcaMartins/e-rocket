#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gyro.hpp>

/**
 * @brief Demo Node for SensorGyro
 */
class DemoSensorGyro : public rclcpp::Node
{
	public:
		explicit DemoSensorGyro() : Node("sensor_gyro")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "sensor_gyro!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::SensorGyro>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::SensorGyro>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_gyro node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorGyro>());

	rclcpp::shutdown();
	return 0;
}
