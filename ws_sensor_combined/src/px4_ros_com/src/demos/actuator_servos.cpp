#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;

/**
 * @brief Demo Node for ActuatorServos
 */
class DemoActuatorServos : public rclcpp::Node
{
	public:
		explicit DemoActuatorServos() : Node("actuator_servos")
		{
			RCLCPP_INFO(this->get_logger(), "actuator_servos!");

			actuator_servos_publisher_ = this->create_publisher<ActuatorServos>("/fmu/in/actuator_servos", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			vehicle_control_mode_publisher_ = this->create_publisher<VehicleControlMode>("/fmu/in/vehicle_control_mode", 10);

			auto timer_callback = [this]() -> void {
				// PX4 will switch out of offboard mode if the stream rate of 
				// OffboardControlMode messages drops below approximately 2Hz
				publish_offboard_control_mode();

				// PX4 requires that the vehicle is already receiving OffboardControlMode messages 
				// before it will arm in offboard mode, 
				// or before it will switch to offboard mode when flying
				if (offboard_setpoint_counter_ == 15) {
					// Change to Offboard mode
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					RCLCPP_INFO(this->get_logger(), "Offboard mode command send");

					// Confirm that we are in offboard mode
					is_offboard_mode_ = true;
					RCLCPP_INFO(this->get_logger(), "Offboard mode confirmed");

					// Arm the vehicle
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
					RCLCPP_INFO(this->get_logger(), "Arm command send");

					// change the vehicle control mode
					this->publish_vehicle_control_mode();
					RCLCPP_INFO(this->get_logger(), "Vehicle control mode command send");
				}

				if (is_offboard_mode_) {
					publish_actuator_servos();
				}

				// disarm the vehicle
				if (offboard_setpoint_counter_ == 200) {
					// Disarm the vehicle
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
					RCLCPP_INFO(this->get_logger(), "Disarm command send");

					// change to Manual mode
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
					RCLCPP_INFO(this->get_logger(), "Manual mode command send");

					is_offboard_mode_ = false;
					RCLCPP_INFO(this->get_logger(), "Offboard mode disabled");
				}
				offboard_setpoint_counter_++;
			};

			timer_ = this->create_wall_timer(100ms, timer_callback);
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<ActuatorServos>::SharedPtr actuator_servos_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleControlMode>::SharedPtr vehicle_control_mode_publisher_;

		uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent

		bool is_offboard_mode_ = false;

		void publish_offboard_control_mode();
		void publish_actuator_servos();
		void publish_vehicle_control_mode();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Publish the offboard control mode.
 *        For this example, only direct actuator is active.
 */
void DemoActuatorServos::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void DemoActuatorServos::publish_actuator_servos()
{
	static double time = 0.0;
	ActuatorServos msg{};

	// Generate sinusoidal values for actuator positions
	for (int i = 0; i < 8; ++i) {
		msg.control[i] = 0.8 * sin((time + i * M_PI) * 2); // Sinusoidal wave between -0.3 and 0.3
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_servos_publisher_->publish(msg);

	time += 0.1; // Increment time for the next wave
}

/**
 * @brief Publish the vehicle control mode.
 *        For this example, we are setting the vehicle to offboard mode.
 */
void DemoActuatorServos::publish_vehicle_control_mode()
{
	VehicleControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.flag_armed = true;

	msg.flag_multicopter_position_control_enabled = false;

	msg.flag_control_manual_enabled = false;
	msg.flag_control_auto_enabled = false;
	msg.flag_control_offboard_enabled = true;
	msg.flag_control_position_enabled = false;
	msg.flag_control_velocity_enabled = false;
	msg.flag_control_altitude_enabled = false;
	msg.flag_control_climb_rate_enabled = false;
	msg.flag_control_acceleration_enabled = false;
	msg.flag_control_attitude_enabled = false;
	msg.flag_control_rates_enabled = false;
	msg.flag_control_allocation_enabled = false;
	msg.flag_control_termination_enabled = false;

	msg.source_id = 1;

	vehicle_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void DemoActuatorServos::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting actuator_servos node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoActuatorServos>());

	rclcpp::shutdown();
	return 0;
}
