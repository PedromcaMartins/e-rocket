#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

/**
 * @brief Demo Node for ActuatorServos
 */
class DemoActuatorServos : public rclcpp::Node
{
	public:
		explicit DemoActuatorServos() : Node("actuator_servos")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "actuator_servos!");
		}

	private:
		rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_servos node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoActuatorServos>());

	rclcpp::shutdown();
	return 0;
}
