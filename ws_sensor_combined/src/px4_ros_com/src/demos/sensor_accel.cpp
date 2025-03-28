#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_accel.hpp>

/**
 * @brief Demo Node for SensorAccel
 */
class DemoSensorAccel : public rclcpp::Node
{
	public:
		explicit DemoSensorAccel() : Node("sensor_accel")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "sensor_accel!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::SensorAccel>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::SensorAccel>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_accel node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorAccel>());

	rclcpp::shutdown();
	return 0;
}
