#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <fstream>

/**
 * @brief Demo Node for testing sensor readings and actutators
 */
class DemoSensorCombined : public rclcpp::Node
{
public:
	explicit DemoSensorCombined() : Node("sensor_combined_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {

			// live data
			std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
			std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
			std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
			std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
			std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
			std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
			std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
			std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
			std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;

			// save live data to file
			log_to_file(msg);
		});

		log_file_.open("log/demos/sensor_combined_log.csv", std::ios::out | std::ios::app);
		if (log_file_.is_open()) {
			log_file_ << "timestamp,gyro_rad[0],gyro_rad[1],gyro_rad[2],gyro_integral_dt,"
					  << "accelerometer_timestamp_relative,accelerometer_m_s2[0],accelerometer_m_s2[1],accelerometer_m_s2[2],accelerometer_integral_dt"
					  << std::endl;
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
		}
	}

	~DemoSensorCombined() {
		if (log_file_.is_open()) {
			log_file_.close();
		}
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
	std::ofstream log_file_;

	void log_to_file(const px4_msgs::msg::SensorCombined::UniquePtr &msg) {
		if (log_file_.is_open()) {
			log_file_ << msg->timestamp << ","
					  << msg->gyro_rad[0] << ","
					  << msg->gyro_rad[1] << ","
					  << msg->gyro_rad[2] << ","
					  << msg->gyro_integral_dt << ","
					  << msg->accelerometer_timestamp_relative << ","
					  << msg->accelerometer_m_s2[0] << ","
					  << msg->accelerometer_m_s2[1] << ","
					  << msg->accelerometer_m_s2[2] << ","
					  << msg->accelerometer_integral_dt  << std::endl;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoSensorCombined>());

	rclcpp::shutdown();
	return 0;
}
