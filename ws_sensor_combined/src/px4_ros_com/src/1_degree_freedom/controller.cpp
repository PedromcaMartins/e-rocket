#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_ros_com/msg/controller_debug.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>

const char CONTROLLER_INPUT_SETPOINT_PARAM[] = "controller_input_setpoint";

using namespace std::chrono;
using namespace px4_msgs::msg;

struct Euler {
    float roll, pitch, yaw;
};

/**
 * @brief Demo Node for offboard control using actuator servos and attitude readings
 */
class Controller : public rclcpp::Node
{
public:
	explicit Controller() : Node("controller")
	{
		actuator_servos_publisher_ = this->create_publisher<ActuatorServos>("/fmu/in/actuator_servos", 10);
		actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_control_mode_publisher_ = this->create_publisher<VehicleControlMode>("/fmu/in/vehicle_control_mode", 10);
        controller_debug_publisher_ = this->create_publisher<px4_ros_com::msg::ControllerDebug>("/offboard/controller_debug", 10);

		pitch_angle_.store(0.0, std::memory_order_relaxed);
		angular_velocity_.store(0.0, std::memory_order_relaxed);
		pitch_angle_setpoint_.store(0.0, std::memory_order_relaxed);
        is_offboard_mode_.store(false, std::memory_order_relaxed);

        state_machine_iteration_ = 0;

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
			[this](const VehicleAttitude::SharedPtr msg) {
				// Process the vehicle attitude message
				auto q = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
				auto euler = quaternionToEulerRadians(q);
				pitch_angle_.store(euler.pitch, std::memory_order_relaxed);
			}
		);

        vehicle_angular_velocity_subscription_ = this->create_subscription<VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
            [this](const VehicleAngularVelocity::SharedPtr msg) {
                auto pitch_angular_velocity = msg->xyz[1];

                angular_velocity_.store(pitch_angular_velocity, std::memory_order_relaxed);
            }
        );

        // Declare the setpoint parameter
        this->declare_parameter<float>(CONTROLLER_INPUT_SETPOINT_PARAM, 0.0f);

        // Set up parameter callback
        setpoint_param_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Controller::setpoint_param_callback, this, std::placeholders::_1)
        );

		state_machine_callback_timer_ = this->create_wall_timer(100ms, // 10 Hz
            [this]() {
                // PX4 will switch out of offboard mode if the stream rate of 
                // OffboardControlMode messages drops below approximately 2Hz
                // PX4 requires that the vehicle is already receiving OffboardControlMode messages 
                // before it will arm in offboard mode, 
                // or before it will switch to offboard mode when flying
                publish_offboard_control_mode();

                // Standby to Mission
                if (state_machine_iteration_ == 15) {
                    // Change to Offboard mode
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    RCLCPP_INFO(this->get_logger(), "Offboard mode command send");

                    // Confirm that we are in offboard mode
                    is_offboard_mode_.store(true, std::memory_order_relaxed);
                    RCLCPP_WARN(this->get_logger(), "Offboard mode not confirmed");

                    // Arm the vehicle
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                    RCLCPP_INFO(this->get_logger(), "Arm command send");

                    // change the vehicle control mode
                    this->publish_vehicle_control_mode();
                    RCLCPP_INFO(this->get_logger(), "Vehicle control mode command send");
                    RCLCPP_WARN(this->get_logger(), "Vehicle control mode not confirmed");
                }

                // Mission to Abort
                // if (state_machine_iteration_ == 1200) {
                //     // Disarm the vehicle
                //     this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                //     RCLCPP_INFO(this->get_logger(), "Disarm command send");

                //     // change to Manual mode
                //     this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
                //     RCLCPP_INFO(this->get_logger(), "Manual mode command send");

                //     // Confirm that we are in manual mode
                //     is_offboard_mode_.store(false, std::memory_order_relaxed);
                //     RCLCPP_WARN(this->get_logger(), "Manual mode not confirmed");
                // }
                state_machine_iteration_++;
            }
        );

        controller_timer_ = this->create_wall_timer(20ms, // 50 Hz
            [this]() {
                // match state in FSM to get mission setpoints
                if (!is_offboard_mode_.load(std::memory_order_relaxed)) {
                    return;
                }

                auto pitch_angle = pitch_angle_.load(std::memory_order_relaxed);
                auto angular_velocity = angular_velocity_.load(std::memory_order_relaxed);
                auto pitch_angle_setpoint = pitch_angle_setpoint_.load(std::memory_order_relaxed);
                auto dt = 0.02f; // 50 Hz

                // Control algorithm
                auto tilt_angle = controller(pitch_angle, angular_velocity, pitch_angle_setpoint, dt);
                publish_controller_debug(pitch_angle, angular_velocity, pitch_angle_setpoint, tilt_angle);

                // Convert to PWM signal
                auto tilt_pwm = tilt_angle;

                publish_actuator_servo(tilt_pwm);

                auto motor_pwm = 0.10f; // 5% duty cycle
                publish_actuator_motors(motor_pwm, motor_pwm);
            }
        );
	}

private:
	rclcpp::TimerBase::SharedPtr state_machine_callback_timer_;
    rclcpp::TimerBase::SharedPtr mission_planner_timer_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Publishers and Subscribers
	rclcpp::Publisher<ActuatorServos>::SharedPtr actuator_servos_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleControlMode>::SharedPtr vehicle_control_mode_publisher_;
    rclcpp::Publisher<px4_ros_com::msg::ControllerDebug>::SharedPtr controller_debug_publisher_;

	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_subscription_;

	uint64_t state_machine_iteration_;   //!< counter for mission timekeeping

	std::atomic<bool> is_offboard_mode_; //!< flag to check if the vehicle is in offboard mode

    // control algorithm variables
	std::atomic<float> pitch_angle_;
	std::atomic<float> angular_velocity_;
	std::atomic<float> pitch_angle_setpoint_;

    //!< Setpoint variable
    OnSetParametersCallbackHandle::SharedPtr setpoint_param_handle_;

    // Add parameter callback method
    rcl_interfaces::msg::SetParametersResult setpoint_param_callback(
        const std::vector<rclcpp::Parameter> &parameters);

	//!< Auxiliary functions
	Euler quaternionToEulerRadians(const Eigen::Quaternionf q);
	void publish_actuator_servo(float tilt_pwm);
	void publish_actuator_motors(float upwards_motor_pwm, float downwards_motor_pwm);

	void publish_offboard_control_mode();
	void publish_vehicle_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    float controller(float pitch_angle, float angular_velocity, float pitch_angle_setpoint, float dt);
    void publish_controller_debug(float pitch_angle, float angular_velocity, float pitch_angle_setpoint, float tilt_angle);
};

rcl_interfaces::msg::SetParametersResult Controller::setpoint_param_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == CONTROLLER_INPUT_SETPOINT_PARAM) {
            float new_setpoint = param.as_double();
            pitch_angle_setpoint_.store(new_setpoint);
            RCLCPP_INFO(this->get_logger(), "Updated setpoint to: %f", new_setpoint);
        }
    }

    return result;
}

void Controller::publish_controller_debug(float pitch_angle, float angular_velocity, float pitch_angle_setpoint, float tilt_angle) {
    RCLCPP_INFO(this->get_logger(), "Pitch angle: %f, Angular velocity: %f, Setpoint: %f, Output: %f", pitch_angle, angular_velocity, pitch_angle_setpoint, tilt_angle);

    px4_ros_com::msg::ControllerDebug msg{};
    msg.pitch_angle = pitch_angle;
    msg.angular_velocity = angular_velocity;
    msg.pitch_angle_setpoint = pitch_angle_setpoint;
    msg.tilt_angle = tilt_angle;
    msg.stamp = this->get_clock()->now();

    controller_debug_publisher_->publish(msg);
}

/**
 * @brief Controller function
 * @param delta_theta Current pitch angle
 * @param delta_omega Current angular velocity
 * @param delta_theta_desired Desired pitch angle setpoint
 * @param dt Time step for the controller
 * @return Tilt angle for the pitch servo
 */
float Controller::controller(float delta_theta, float delta_omega, float delta_theta_desired, float dt)
{
    const float k_p = 1.8275f; // Proportional gain
    const float k_d = 0.8273f; // Derivative gain
    const float k_i = 1.4142f; // Integral gain

    // Update the integrated error
    static float zeta_theta = 0.0f;
    zeta_theta = zeta_theta + (delta_theta_desired - delta_theta) * dt; 

    // Compute control input
    float dot_product = delta_theta * k_p + delta_omega * k_d;
    float delta_gamma = -dot_product + zeta_theta * k_i;

    return delta_gamma;
}

/**
 * @brief Convert quaternion to Euler angles (radiands)
 * @param q Quaternion
 * @return Euler angles (roll, pitch, yaw)
 */
Euler Controller::quaternionToEulerRadians(const Eigen::Quaternionf q) {
    auto w = q.w();
    auto x = q.x();
    auto y = q.y();
    auto z = q.z();

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);
    float pitch = 0.0;

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(90.0, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void Controller::publish_actuator_servo(float tilt_pwm)
{
    if (tilt_pwm > 1.0f) {
        tilt_pwm = 1.0f;
    } else if (tilt_pwm < -1.0f) {
        tilt_pwm = -1.0f;
    }
	ActuatorServos msg{};
	msg.control[1] = -tilt_pwm;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_servos_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motors.
 *        For this example, we are generating sinusoidal values for the actuator positions.
 */
void Controller::publish_actuator_motors(float upwards_motor_pwm, float downwards_motor_pwm)
{
	ActuatorMotors msg{};

	msg.control[0] = upwards_motor_pwm;
	msg.control[1] = downwards_motor_pwm;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	actuator_motors_publisher_->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only direct actuator is active.
 */
void Controller::publish_offboard_control_mode()
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
void Controller::publish_vehicle_control_mode()
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
void Controller::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	rclcpp::spin(std::make_shared<Controller>());

	rclcpp::shutdown();
	return 0;
}
