#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;

/**
 * @brief Demo Node for ActuatorMotors
 */
class DemoActuatorMotors : public rclcpp::Node
{
	public:
		explicit DemoActuatorMotors() : Node("actuator_motors")
		{
			RCLCPP_INFO(this->get_logger(), "actuator_motors!");

			actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			vehicle_control_mode_publisher_ = this->create_publisher<VehicleControlMode>("/fmu/in/vehicle_control_mode", 10);

			offboard_setpoint_counter_ = 0;

			auto timer_callback = [this]() -> void {
				// PX4 will switch out of offboard mode if the stream rate of 
				// OffboardControlMode messages drops below approximately 2Hz
				publish_offboard_control_mode();
				// Always has to be paired with actuator_motors
				publish_actuator_motors();

				// PX4 requires that the vehicle is already receiving OffboardControlMode messages 
				// before it will arm in offboard mode, 
				// or before it will switch to offboard mode when flying
				if (offboard_setpoint_counter_ == 15) {
					// Change to Offboard mode
					this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					RCLCPP_INFO(this->get_logger(), "Offboard mode command send");

					// Arm the vehicle
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
					RCLCPP_INFO(this->get_logger(), "Arm command send");

					// change the vehicle control mode
					this->publish_vehicle_control_mode();
					RCLCPP_INFO(this->get_logger(), "Vehicle control mode command send");
				}

				// disarm the vehicle
				if (offboard_setpoint_counter_ == 200) {
					// Disarm the vehicle
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
					RCLCPP_INFO(this->get_logger(), "Disarm command send");

					// change to Manual mode
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
					RCLCPP_INFO(this->get_logger(), "Manual mode command send");
				}
				offboard_setpoint_counter_++;
			};

			timer_ = this->create_wall_timer(100ms, timer_callback);
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleControlMode>::SharedPtr vehicle_control_mode_publisher_;

		uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

		void publish_offboard_control_mode();
		void publish_actuator_motors();
		void publish_vehicle_control_mode();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Publish the offboard control mode.
 *        For this example, only direct actuator is active.
 */
void DemoActuatorMotors::publish_offboard_control_mode()
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
void DemoActuatorMotors::publish_actuator_motors()
{
	static double time = 0.0;
	ActuatorMotors msg{};

	// Generate sinusoidal values for actuator positions
	for (int i = 0; i < 12; ++i) {
		msg.control[i] = 0.25 * (1.0 + sin(time + i * M_PI / 4)); // Sinusoidal wave between 0 and 0.5
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_motors_publisher_->publish(msg);

	time += 0.1; // Increment time for the next wave
}

/**
 * @brief Publish the vehicle control mode.
 *        For this example, we are setting the vehicle to offboard mode.
 */
void DemoActuatorMotors::publish_vehicle_control_mode()
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
void DemoActuatorMotors::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting actuator_motors node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemoActuatorMotors>());

	rclcpp::shutdown();
	return 0;
}
