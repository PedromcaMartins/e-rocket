#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/hello_world.hpp>

/**
 * @brief Demo Node for HelloWorld
 */
class DemoHelloWorld : public rclcpp::Node
{
	public:
		explicit DemoHelloWorld() : Node("hello_world")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "hello_world!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::HelloWorld>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::HelloWorld>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting hello_world node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoHelloWorld>());

	rclcpp::shutdown();
	return 0;
}
