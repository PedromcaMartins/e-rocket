#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;

struct Quaternion {
    float w, x, y, z;
};

/**
 * @brief Demo Node for offboard control using actuator servos and attitude readings
 */
class OffboardControl : public rclcpp::Node
{
public:
	explicit OffboardControl() : Node("offboard_control")
	{
		actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
		actuator_servos_publisher_ = this->create_publisher<ActuatorServos>("/fmu/in/actuator_servos", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_control_mode_publisher_ = this->create_publisher<VehicleControlMode>("/fmu/in/vehicle_control_mode", 10);

		roll_.store(0.0, std::memory_order_relaxed);
		pitch_.store(0.0, std::memory_order_relaxed);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
			[this](const VehicleAttitude::SharedPtr msg) {
				// Process the vehicle attitude message
				Quaternion q;
				q.w = msg->q[0];
				q.x = msg->q[1];
				q.y = msg->q[2];
				q.z = msg->q[3];

				float roll, pitch, yaw;
				quaternionToEuler(q, roll, pitch, yaw);

				// Map roll and pitch from [-90, 90] to [-1, 1] but constrained to [-0.75, 0.75]
				roll_.store(std::max(-0.75f, std::min(0.75f, roll / 90.0f)), std::memory_order_relaxed);
				pitch_.store(std::max(-0.75f, std::min(0.75f, pitch / 90.0f)), std::memory_order_relaxed);

				if (is_offboard_mode_) {
					publish_actuator_servos();
					publish_actuator_motors();
				}
			}
		);

		auto timer_callback = [this]() -> void {
			// PX4 will switch out of offboard mode if the stream rate of 
			// OffboardControlMode messages drops below approximately 2Hz
			publish_offboard_control_mode();

			// PX4 requires that the vehicle is already receiving OffboardControlMode messages 
			// before it will arm in offboard mode, 
			// or before it will switch to offboard mode when flying
			if (timer_callback_iteration_ == 15) {
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

			// disarm the vehicle
			if (timer_callback_iteration_ == 300) {
				// Disarm the vehicle
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
				RCLCPP_INFO(this->get_logger(), "Disarm command send");

				// change to Manual mode
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
				RCLCPP_INFO(this->get_logger(), "Manual mode command send");

				is_offboard_mode_ = false;
				RCLCPP_INFO(this->get_logger(), "Offboard mode disabled");
			}
			timer_callback_iteration_++;
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Publisher<ActuatorServos>::SharedPtr actuator_servos_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleControlMode>::SharedPtr vehicle_control_mode_publisher_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

	uint64_t timer_callback_iteration_ = 0;   //!< counter for the number of setpoints sent

	bool is_offboard_mode_ = false;

	std::atomic<float> roll_;   //!< common synced roll position for servos
	std::atomic<float> pitch_;   //!< common synced pitch position for servos

	void quaternionToEuler(const Quaternion& q, float& roll, float& pitch, float& yaw);
	void publish_actuator_servos();
	void publish_actuator_motors();

	void publish_offboard_control_mode();
	void publish_vehicle_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::quaternionToEuler(const Quaternion& q, float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(90.0, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp) * 180.0 / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void OffboardControl::publish_actuator_servos()
{
	static double time = 0.0;
	ActuatorServos msg{};

	msg.control[0] = pitch_.load(); // Pitch value between -0.75 and 0.75
	msg.control[1] = -roll_.load(); // Roll value between -0.75 and 0.75

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_servos_publisher_->publish(msg);

	time += 0.1; // Increment time for the next wave
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void OffboardControl::publish_actuator_motors()
{
	static double time = 0.0;
	ActuatorMotors msg{};

	// Generate sinusoidal values for actuator positions
	float value_1 = 0.05f * (sin(time / 10.0f)); // Sinusoidal wave between 0 and 0.10
	float value_2 = -value_1;

	if (value_1 < 0.0f) {
		value_1 = NAN;
	}
	if (value_2 < 0.0f) {
		value_2 = NAN;
	}

	msg.control[0] = value_1;
	msg.control[1] = value_2;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_motors_publisher_->publish(msg);

	time += 0.1; // Increment time for the next wave
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only direct actuator is active.
 */
void OffboardControl::publish_offboard_control_mode()
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
 * @brief Publish the vehicle control mode.
 *        For this example, we are setting the vehicle to offboard mode.
 */
void OffboardControl::publish_vehicle_control_mode()
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
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
