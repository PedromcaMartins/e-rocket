#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/msg/controller_input_setpoint.hpp>
#include <one_degree_freedom/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;

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

        // Declare the setpoint parameter
        this->declare_parameter<float>(CONTROLLER_INPUT_SETPOINT_PARAM, 0.0f);

        // Set up parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControllerTestNode::parameter_callback, this, std::placeholders::_1)
        );
	}

private:
	//!< Publishers and Subscribers
	rclcpp::Publisher<ControllerInputSetpoint>::SharedPtr setpoint_publisher_;

	//!< Auxiliary functions
    void publish_setpoint(float setpoint_radians);

    //!< Setpoint variable
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Add parameter callback method
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters);
};

void ControllerTestNode::publish_setpoint(float setpoint_radians)
{
    ControllerInputSetpoint msg{};
    msg.stamp = this->get_clock()->now();
    msg.setpoint_radians = setpoint_radians;
    setpoint_publisher_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult ControllerTestNode::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == CONTROLLER_INPUT_SETPOINT_PARAM) {
            float new_setpoint_radians = param.as_double();
            publish_setpoint(new_setpoint_radians);
            RCLCPP_INFO(this->get_logger(), "Updated setpoint to: %f", new_setpoint_radians);
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
