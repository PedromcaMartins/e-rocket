#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_setpoint.hpp>
#include <one_degree_freedom/msg/flight_mode.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::mission;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::flight_mode;

/**
 * @brief Node that tests the controller for a 1-degree-of-freedom system, verifying if the setpoints are being reached. 
 */
class ControllerTestNode : public rclcpp::Node
{
public:
	explicit ControllerTestNode() : Node("controller_test")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        setpoint_publisher_ = this->create_publisher<ControllerInputSetpoint>(
            CONTROLLER_INPUT_SETPOINT_TOPIC, qos
        );

        // Declare the setpoint parameter as array of 3 floats [roll, pitch, yaw]
        this->declare_parameter<std::vector<double>>(
            MISSION_SETPOINT_PARAM, 
            std::vector<double>{0.0, 0.0, 0.0}
        );

        // Set up parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControllerTestNode::parameter_callback, this, std::placeholders::_1)
        );

        flight_mode_get_publisher_ = this->create_publisher<FlightMode>(
            FLIGHT_MODE_GET_TOPIC, qos
        );

        flight_mode_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                FlightMode msg {};
                msg.stamp = this->get_clock()->now();
                msg.flight_mode = FlightMode::IN_MISSION;
                flight_mode_get_publisher_->publish(msg);
            }
        );
	}

private:
	//!< Publishers and Subscribers
	rclcpp::Publisher<ControllerInputSetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Publisher<FlightMode>::SharedPtr flight_mode_get_publisher_;

    rclcpp::TimerBase::SharedPtr flight_mode_timer_;

	//!< Auxiliary functions
    void publish_setpoint(float roll_setpoint_radians, float pitch_setpoint_radians, float yaw_setpoint_radians);

    //!< Setpoint variable
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Add parameter callback method
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters);
};

void ControllerTestNode::publish_setpoint(float roll_setpoint_radians, float pitch_setpoint_radians, float yaw_setpoint_radians)
{
    ControllerInputSetpoint msg{};
    msg.stamp = this->get_clock()->now();
    msg.roll_setpoint_radians = roll_setpoint_radians;
    msg.pitch_setpoint_radians = pitch_setpoint_radians;
    msg.yaw_setpoint_radians = yaw_setpoint_radians;
    setpoint_publisher_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult ControllerTestNode::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == MISSION_SETPOINT_PARAM) {
            std::vector<double> new_setpoint_radians = param.as_double_array();

            if (
                new_setpoint_radians.size() != 3 || 
                new_setpoint_radians[0] == NAN ||
                new_setpoint_radians[1] == NAN || 
                new_setpoint_radians[2] == NAN
            ) {
                RCLCPP_ERROR(this->get_logger(), "Invalid setpoint values, must be finite numbers.");
                result.successful = false;
                result.reason = "Invalid setpoint values";
                return result;
            }

            publish_setpoint(
                new_setpoint_radians[0],
                new_setpoint_radians[1],
                new_setpoint_radians[2]
            );

            RCLCPP_INFO(this->get_logger(), 
                "Updated setpoint to: [%f, %f, %f]",
                new_setpoint_radians[0],
                new_setpoint_radians[1],
                new_setpoint_radians[2]
            );
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting controller test node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControllerTestNode>());

	rclcpp::shutdown();
	return 0;
}
