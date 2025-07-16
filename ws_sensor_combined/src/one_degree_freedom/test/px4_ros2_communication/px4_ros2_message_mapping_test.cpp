#include <rclcpp/rclcpp.hpp>

#include <one_degree_freedom/msg/controller_output_servo_tilt_angle.hpp>
#include <one_degree_freedom/msg/controller_output_motor_thrust.hpp>
#include <one_degree_freedom/msg/controller_input_attitude.hpp>
#include <one_degree_freedom/msg/controller_input_angular_rate.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <one_degree_freedom/frame_transforms.h>
#include <one_degree_freedom/constants.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::constants::controller;
using namespace one_degree_freedom::constants::px4_ros2_message_mapping;
using namespace one_degree_freedom::msg;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class Px4Ros2MessageMappingTest : public rclcpp::Node
{
public: 
    Px4Ros2MessageMappingTest() : 
		Node("px4_ros2_flight_mode_test"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
	    
        roll_radians_{0.0f},
	    x_roll_angular_rate_radians_per_second_{0.0f},
        pitch_radians_{0.0f},
	    y_pitch_angular_rate_radians_per_second_{0.0f},
        yaw_radians_{0.0f},
	    z_yaw_angular_rate_radians_per_second_{0.0f},

        controller_output_servo_tilt_angle_publisher_{this->create_publisher<ControllerOutputServoTiltAngle>(
            CONTROLLER_OUTPUT_SERVO_TILT_ANGLE_TOPIC, qos_
        )},
        controller_output_motor_thrust_publisher_{this->create_publisher<ControllerOutputMotorThrust>(
            CONTROLLER_OUTPUT_MOTOR_THRUST_TOPIC, qos_
        )},
        controller_input_attitude_subscription_{this->create_subscription<ControllerInputAttitude>(
            CONTROLLER_INPUT_ATTITUDE_TOPIC, qos_, 
            [this](const ControllerInputAttitude::SharedPtr msg) {
                roll_radians_.store(msg->roll_radians);
                pitch_radians_.store(msg->pitch_radians);
                yaw_radians_.store(msg->yaw_radians);
            }
        )},
        controller_input_angular_rate_subscription_{this->create_subscription<ControllerInputAngularRate>(
            CONTROLLER_INPUT_ANGULAR_RATE_TOPIC, qos_, 
            [this](const ControllerInputAngularRate::SharedPtr msg) {
                x_roll_angular_rate_radians_per_second_.store(msg->x_roll_angular_rate_radians_per_second);
                y_pitch_angular_rate_radians_per_second_.store(msg->y_pitch_angular_rate_radians_per_second);
                z_yaw_angular_rate_radians_per_second_.store(msg->z_yaw_angular_rate_radians_per_second);
            }
        )},
        mocap_publisher_{this->create_publisher<geometry_msgs::msg::PoseStamped>(
            MOCAP_TOPIC, qos_
        )},
        mocap_euler_publisher_{this->create_publisher<geometry_msgs::msg::Vector3>(
            "/offboard/mocap_enu_debug", qos_
        )},
        vehicle_mocap_odometry_subscription_{this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_mocap_odometry", qos_,
            std::bind(&Px4Ros2MessageMappingTest::mocap_pose_callback, this, std::placeholders::_1)
        )},
        vehicle_mocap_euler_publisher_{this->create_publisher<geometry_msgs::msg::Vector3>(
            "/offboard/mocap_px4_debug", qos_
        )}
    {
        test_timer_ = this->create_wall_timer(100ms,
            [this]() {
                static double time = 0.0;
                time++;
                float motor_sin_wave = 0.05f * (sin(time / 10.0f));

                if (motor_sin_wave < 0.0f) {
                    motor_sin_wave = NAN;
                }

                float servo_sin_wave = 0.5f * (sin(time / 10.0f));

                publish_controller_output_servo_tilt_angle_(servo_sin_wave, servo_sin_wave);
                publish_controller_output_motor_thrust_(motor_sin_wave, motor_sin_wave);
                RCLCPP_INFO(this->get_logger(), 
                    "roll: %f, pitch: %f, yaw: %f", 
                    roll_radians_.load(),
                    pitch_radians_.load(),
                    yaw_radians_.load()
                );
                RCLCPP_INFO(this->get_logger(), 
                    "ang rate pitch: %f, ang rate roll: %f, ang rate yaw: %f", 
                    x_roll_angular_rate_radians_per_second_.load(),
                    y_pitch_angular_rate_radians_per_second_.load(),
                    z_yaw_angular_rate_radians_per_second_.load()
                );

                send_mocap_pose_enu();
            }
		);
    }

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

	std::atomic<float> roll_radians_;
	std::atomic<float> x_roll_angular_rate_radians_per_second_;

    std::atomic<float> pitch_radians_;
	std::atomic<float> y_pitch_angular_rate_radians_per_second_;

    std::atomic<float> yaw_radians_;
	std::atomic<float> z_yaw_angular_rate_radians_per_second_;

    rclcpp::Publisher<ControllerOutputServoTiltAngle>::SharedPtr controller_output_servo_tilt_angle_publisher_;
    rclcpp::Publisher<ControllerOutputMotorThrust>::SharedPtr controller_output_motor_thrust_publisher_;
    rclcpp::Subscription<ControllerInputAttitude>::SharedPtr controller_input_attitude_subscription_;
    rclcpp::Subscription<ControllerInputAngularRate>::SharedPtr controller_input_angular_rate_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mocap_euler_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_mocap_odometry_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vehicle_mocap_euler_publisher_;

	rclcpp::TimerBase::SharedPtr test_timer_;

    void publish_controller_output_servo_tilt_angle_(float outer_servo_tilt_angle_radians, float inner_servo_tilt_angle_radians);
    void publish_controller_output_motor_thrust_(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage);
    void send_mocap_pose_enu();

    void mocap_pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publish_mocap_euler_angles(const Eigen::Quaterniond &q);
    void publish_vehicle_mocap_euler_angles(const Eigen::Quaterniond &q);
};

void Px4Ros2MessageMappingTest::send_mocap_pose_enu() {
    geometry_msgs::msg::PoseStamped mocap_pose_enu_msg;
    mocap_pose_enu_msg.header.stamp = this->get_clock()->now();
    mocap_pose_enu_msg.header.frame_id = "mocap_frame";

    // Set a dummy pose for testing
    mocap_pose_enu_msg.pose.position.x = 1.0;
    mocap_pose_enu_msg.pose.position.y = 2.0;
    mocap_pose_enu_msg.pose.position.z = 3.0;

    // TODO: Send sinusoidal orientation for testing
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;

    auto q = Eigen::Quaterniond(
        cos(yaw / 2) * cos(pitch / 2) * cos(roll / 2) + sin(yaw / 2) * sin(pitch / 2) * sin(roll / 2),
        cos(yaw / 2) * cos(pitch / 2) * sin(roll / 2) - sin(yaw / 2) * sin(pitch / 2) * cos(roll / 2),
        cos(yaw / 2) * sin(pitch / 2) * cos(roll / 2) + sin(yaw / 2) * cos(pitch / 2) * sin(roll / 2),
        sin(yaw / 2) * cos(pitch / 2) * cos(roll / 2) - cos(yaw / 2) * sin(pitch / 2) * sin(roll / 2)
    );

    mocap_pose_enu_msg.pose.orientation.w = q.w();
    mocap_pose_enu_msg.pose.orientation.x = q.x();
    mocap_pose_enu_msg.pose.orientation.y = q.y();
    mocap_pose_enu_msg.pose.orientation.z = q.z();

    mocap_publisher_->publish(mocap_pose_enu_msg);

    RCLCPP_INFO(this->get_logger(), "Published Mocap Pose ENU: [%f, %f, %f]",
        mocap_pose_enu_msg.pose.position.x,
        mocap_pose_enu_msg.pose.position.y,
        mocap_pose_enu_msg.pose.position.z
    );

    RCLCPP_INFO(this->get_logger(), "Published Mocap Orientation ENU: [%lf, %lf, %lf]", roll, pitch, yaw);

    publish_mocap_euler_angles(Eigen::Quaterniond(
        mocap_pose_enu_msg.pose.orientation.w,
        mocap_pose_enu_msg.pose.orientation.x,
        mocap_pose_enu_msg.pose.orientation.y,
        mocap_pose_enu_msg.pose.orientation.z
    ));
}

void Px4Ros2MessageMappingTest::publish_controller_output_servo_tilt_angle_(float outer_servo_tilt_angle_radians, float inner_servo_tilt_angle_radians) {
    ControllerOutputServoTiltAngle msg {};
    msg.outer_servo_tilt_angle_radians = outer_servo_tilt_angle_radians;
    msg.inner_servo_tilt_angle_radians = inner_servo_tilt_angle_radians;
    msg.stamp = this->get_clock()->now();
    controller_output_servo_tilt_angle_publisher_->publish(msg);
}

void Px4Ros2MessageMappingTest::publish_controller_output_motor_thrust_(float upwards_motor_thrust_percentage, float downwards_motor_thrust_percentage) {
    ControllerOutputMotorThrust msg {};
    msg.upwards_motor_thrust_percentage = upwards_motor_thrust_percentage;
    msg.downwards_motor_thrust_percentage = downwards_motor_thrust_percentage;
    msg.stamp = this->get_clock()->now();
    controller_output_motor_thrust_publisher_->publish(msg);
}

void Px4Ros2MessageMappingTest::mocap_pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    publish_vehicle_mocap_euler_angles(Eigen::Quaterniond(
        msg->q[0],
        msg->q[1],
        msg->q[2],
        msg->q[3]
    ));
}

void Px4Ros2MessageMappingTest::publish_mocap_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_radians(q);
    auto to_degrees = [](float radians) {
        return radians * (180.0f / M_PI);
    };
    geometry_msgs::msg::Vector3 euler_msg;
    euler_msg.x = to_degrees(euler.roll);
    euler_msg.y = to_degrees(euler.pitch);
    euler_msg.z = to_degrees(euler.yaw);
    mocap_euler_publisher_->publish(euler_msg);
}

void Px4Ros2MessageMappingTest::publish_vehicle_mocap_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_radians(q);
    auto to_degrees = [](float radians) {
        return radians * (180.0f / M_PI);
    };
    geometry_msgs::msg::Vector3 euler_msg;
    euler_msg.x = to_degrees(euler.roll);
    euler_msg.y = to_degrees(euler.pitch);
    euler_msg.z = to_degrees(euler.yaw);
    vehicle_mocap_euler_publisher_->publish(euler_msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Message Mapping Test node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Ros2MessageMappingTest>());

	rclcpp::shutdown();
	return 0;
}
