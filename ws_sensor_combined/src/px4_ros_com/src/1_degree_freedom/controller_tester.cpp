#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/msg/controller_input_attitude.hpp>
#include <px4_ros_com/msg/controller_input_angular_rate.hpp>
#include <px4_ros_com/msg/controller_input_setpoint.hpp>
#include <px4_ros_com/msg/controller_output_tilt_angle.hpp>
#include <px4_ros_com/constants.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_ros_com::msg;
using namespace px4_ros_com::constants::one_degree_of_freedom;

/**
 * @brief Node that tests the controller for a 1-degree-of-freedom system, using simulated data
 */
class ControllerTester : public rclcpp::Node
{
public:
	explicit ControllerTester() : Node("controller_tester")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        tilt_angle_subscriber_ = this->create_subscription<ControllerOutputTiltAngle>(
            CONTROLLER_OUTPUT_TILT_ANGLE, qos,
            std::bind(&ControllerTester::controller_output_callback, this, std::placeholders::_1)
        );

        attitude_publisher_ = this->create_publisher<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE, qos
        );

        angular_rate_publisher_ = this->create_publisher<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE, qos
        );

        setpoint_publisher_ = this->create_publisher<ControllerInputSetpoint>(
            CONTROLLER_INPUT_SETPOINT, qos
        );

        // Declare the setpoint parameter
        this->declare_parameter<float>(CONTROLLER_INPUT_SETPOINT_PARAM, 0.0f);

        // Set up parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControllerTester::parameter_callback, this, std::placeholders::_1)
        );
	}

private:
	//!< Publishers and Subscribers
	rclcpp::Publisher<ControllerInputAttitude>::SharedPtr       attitude_publisher_;
	rclcpp::Publisher<ControllerInputAngularRate>::SharedPtr    angular_rate_publisher_;
	rclcpp::Publisher<ControllerInputSetpoint>::SharedPtr       setpoint_publisher_;
	rclcpp::Subscription<ControllerOutputTiltAngle>::SharedPtr  tilt_angle_subscriber_;

    //!< State variables - to be updated by the simulator
    float delta_theta_ = 0.0f; // pitch angle
    float delta_omega_ = 0.0f; // angular position

    //!< Setpoint variable
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	//!< Auxiliary functions
    void publish_attitude(float attitude);
    void publish_angular_rate(float angular_rate);
    void publish_setpoint(float setpoint);
    void controller_output_callback(const ControllerOutputTiltAngle::SharedPtr msg);

    // Add parameter callback method
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters);
};

void ControllerTester::publish_attitude(float attitude)
{
    ControllerInputAttitude msg{};
    msg.stamp = this->get_clock()->now();
    msg.attitude = attitude;
    attitude_publisher_->publish(msg);
}

void ControllerTester::publish_angular_rate(float angular_rate)
{
    ControllerInputAngularRate msg{};
    msg.stamp = this->get_clock()->now();
    msg.angular_rate = angular_rate;
    angular_rate_publisher_->publish(msg);
}

void ControllerTester::publish_setpoint(float setpoint)
{
    ControllerInputSetpoint msg{};
    msg.stamp = this->get_clock()->now();
    msg.setpoint = setpoint;
    setpoint_publisher_->publish(msg);
}

void ControllerTester::controller_output_callback(const ControllerOutputTiltAngle::SharedPtr msg)
{
    float delta_gamma = msg->tilt_angle;
    float step = CONTROLLER_DT;

    float a = delta_gamma * M * L * G / J;
    delta_omega_ += a * step;
    delta_theta_ += delta_omega_ * step;

    // Publish the new state variables
    publish_attitude(delta_theta_);
    publish_angular_rate(delta_omega_);
}

rcl_interfaces::msg::SetParametersResult ControllerTester::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == CONTROLLER_INPUT_SETPOINT_PARAM) {
            float new_setpoint = param.as_double();
            publish_setpoint(new_setpoint);
            RCLCPP_INFO(this->get_logger(), "Updated setpoint to: %f", new_setpoint);
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting controller tester node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControllerTester>());

	rclcpp::shutdown();
	return 0;
}
