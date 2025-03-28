#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

/**
 * @brief Demo Node for VehicleAngularVelocity
 */
class DemoVehicleAngularVelocity : public rclcpp::Node
{
	public:
		explicit DemoVehicleAngularVelocity() : Node("vehicle_angular_velocity")
		{
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			RCLCPP_INFO(this->get_logger(), "vehicle_angular_velocity!");
		}

	private:
		rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr subscription_;
		rclcpp::Publisher<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_angular_velocity node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoVehicleAngularVelocity>());

	rclcpp::shutdown();
	return 0;
}
