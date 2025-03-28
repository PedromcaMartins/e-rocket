#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>

/**
 * @brief Demo Node for ActuatorMotors
 */
class DemoActuatorMotors : public rclcpp::Node
{
	public:
		explicit DemoActuatorMotors() : Node("actuator_motors")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "actuator_motors!");
		}

	private:
		rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_motors node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoActuatorMotors>());

	rclcpp::shutdown();
	return 0;
}
