#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <one_degree_freedom/msg/attitude_controller_debug.hpp>
#include <one_degree_freedom/msg/position_controller_debug.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>

#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;
using namespace one_degree_freedom::msg;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants;
using namespace one_degree_freedom::frame_transforms;

/**
 * @brief Node that simulates the effects of the controller on the environment. It is used for testing the controller
 */
class Simulator : public rclcpp::Node
{
public:
	explicit Simulator() : Node("simulator")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        attitude_controller_debug_subscriber_ = this->create_subscription<AttitudeControllerDebug>(
            CONTROLLER_ATTITUDE_DEBUG_TOPIC, qos,
            std::bind(&Simulator::attitude_controller_output_callback, this, std::placeholders::_1)
        );

        position_controller_debug_subscriber_ = this->create_subscription<PositionControllerDebug>(
            CONTROLLER_POSITION_DEBUG_TOPIC, qos,
            std::bind(&Simulator::position_controller_output_callback, this, std::placeholders::_1)
        );

        attitude_publisher_ = this->create_publisher<VehicleAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos
        );

        angular_rate_publisher_ = this->create_publisher<VehicleAngularVelocity>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos
        );

        local_position_publisher_ = this->create_publisher<VehicleLocalPosition>(
            CONTROLLER_INPUT_LOCAL_POSITION_TOPIC, qos
        );

        this->declare_parameter<double>(CONTROLLER_ATTITUDE_FREQUENCY_HERTZ_PARAM);
        auto controllers_freq = this->get_parameter(CONTROLLER_ATTITUDE_FREQUENCY_HERTZ_PARAM).as_double();
        attitude_time_step_seconds_ = 1.0 / controllers_freq;

        this->declare_parameter<double>(CONTROLLER_POSITION_FREQUENCY_HERTZ_PARAM);
        auto position_controllers_freq = this->get_parameter(CONTROLLER_POSITION_FREQUENCY_HERTZ_PARAM).as_double();
        position_time_step_seconds_ = 1.0 / position_controllers_freq;

        this->declare_parameter<double>(MASS_OF_SYSTEM);
        this->declare_parameter<double>(LENGTH_OF_PENDULUM);
        this->declare_parameter<double>(GRAVITATIONAL_ACCELERATION);
        this->declare_parameter<double>(MOMENT_OF_INERTIA);

        m_ = this->get_parameter(MASS_OF_SYSTEM).as_double();
        l_ = this->get_parameter(LENGTH_OF_PENDULUM).as_double();
        g_ = this->get_parameter(GRAVITATIONAL_ACCELERATION).as_double();
        j_ = this->get_parameter(MOMENT_OF_INERTIA).as_double();

        // Safety check
        if (m_ <= 0.0f || l_ <= 0.0f || g_ <= 0.0f || j_ <= 0.0f || std::isnan(m_) || std::isnan(l_) || std::isnan(g_) || std::isnan(j_)) {
            RCLCPP_ERROR(this->get_logger(), "Could not read simulator parameters correctly.");
            throw std::runtime_error("Simulator parameters invalid");
        }

        RCLCPP_INFO(this->get_logger(), "Simulator parameters: m = %f, l = %f, g = %f, j = %f", m_, l_, g_, j_);

        rclcpp::sleep_for(std::chrono::seconds(1));
        publish_attitude();
	}

private:
	//!< Publishers and Subscribers
	rclcpp::Publisher<VehicleAttitude>::SharedPtr        attitude_publisher_;
	rclcpp::Publisher<VehicleAngularVelocity>::SharedPtr angular_rate_publisher_;
	rclcpp::Publisher<VehicleLocalPosition>::SharedPtr   local_position_publisher_;
	rclcpp::Subscription<AttitudeControllerDebug>::SharedPtr attitude_controller_debug_subscriber_;
	rclcpp::Subscription<PositionControllerDebug>::SharedPtr position_controller_debug_subscriber_;

    //!< State variables - to be updated by the simulator
    double roll_angle_ = 0.0f;         // pitch angle
    double roll_angular_rate_ = 0.0f;  // angular position

    double pitch_angle_ = 0.0f;        // pitch angle
    double pitch_angular_rate_ = 0.0f; // angular position

    double yaw_angle_ = 1.0f;          // pitch angle
    double yaw_angular_rate_ = 0.0f;   // angular position

    Eigen::Vector3d position_     = Eigen::Vector3d::Zero(); // position in the world frame
    Eigen::Vector3d velocity_     = Eigen::Vector3d::Zero(); // velocity in the world frame
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero(); // acceleration in the world frame

    double m_;
    double l_;
    double g_;
    double j_;

    double attitude_time_step_seconds_;
    double position_time_step_seconds_;

	//!< Auxiliary functions
    void publish_attitude();
    void publish_angular_rate();
    void publish_local_position();
    void attitude_controller_output_callback(const AttitudeControllerDebug::SharedPtr msg);
    void position_controller_output_callback(const PositionControllerDebug::SharedPtr msg);
};

void Simulator::publish_attitude()
{
    VehicleAttitude msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    Eigen::Quaterniond q = euler_radians_to_quaternion(
        EulerAngle { roll_angle_, pitch_angle_, yaw_angle_ }
    );

    msg.q[0] = q.w();
    msg.q[1] = q.x();
    msg.q[2] = q.y();
    msg.q[3] = q.z();
    attitude_publisher_->publish(msg);
}

void Simulator::publish_angular_rate()
{
    VehicleAngularVelocity msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.xyz[0] = roll_angular_rate_;
    msg.xyz[1] = pitch_angular_rate_;
    msg.xyz[2] = yaw_angular_rate_;
    angular_rate_publisher_->publish(msg);
}

void Simulator::publish_local_position()
{
    VehicleLocalPosition msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.x = position_[0];
    msg.y = position_[1];
    msg.z = position_[2];

    msg.vx = velocity_[0];
    msg.vy = velocity_[1];
    msg.vz = velocity_[2];

    msg.ax = acceleration_[0];
    msg.ay = acceleration_[1];
    msg.az = acceleration_[2];

    local_position_publisher_->publish(msg);
}

void Simulator::attitude_controller_output_callback(const AttitudeControllerDebug::SharedPtr msg)
{
    double roll_inner_servo_tilt_angle = -msg->roll_inner_servo_tilt_angle;
    double pitch_outer_servo_tilt_angle = -msg->pitch_outer_servo_tilt_angle;
    double yaw_delta_motor_pwm = -msg->yaw_delta_motor_pwm;
    double step = attitude_time_step_seconds_;

    //*********//
    //* roll  *//
    //*********//
    {
        double a = roll_inner_servo_tilt_angle * m_ * l_ * g_ / j_;
        roll_angular_rate_ += a * step;
        roll_angle_ += roll_angular_rate_ * step;
    }

    //*********//
    //* pitch *//
    //*********//
    {
        double a = pitch_outer_servo_tilt_angle * m_ * l_ * g_ / j_;
        pitch_angular_rate_ += a * step;
        pitch_angle_ += pitch_angular_rate_ * step;
    }

    //*********//
    //*  yaw  *//
    //*********//
    {
        double a = yaw_delta_motor_pwm * m_ * l_ * g_ / j_;
        yaw_angular_rate_ += a * step;
        yaw_angle_ += yaw_angular_rate_ * step;
    }

    // Publish the new state variables
    publish_attitude();
    publish_angular_rate();
}

void Simulator::position_controller_output_callback(const PositionControllerDebug::SharedPtr msg)
{
    Eigen::Vector3d desired_acceleration_(
        msg->desired_acceleration[0],
        msg->desired_acceleration[1],
        msg->desired_acceleration[2]
    );
    desired_acceleration_[2] += g_;

    acceleration_ = desired_acceleration_;
    velocity_ += acceleration_ * position_time_step_seconds_;
    position_ += velocity_ * position_time_step_seconds_;

    // Publish the new state variables
    publish_local_position();
}

int main(int argc, char *argv[])
{
	std::cout << "Starting simulator..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Simulator>());

	rclcpp::shutdown();
	return 0;
}
