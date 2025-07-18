#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <one_degree_freedom/constants.hpp>

#include <one_degree_freedom/frame_transforms.h>
#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::frame_transforms;
using namespace one_degree_freedom::constants::mocap_forwarder;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class MocapForwarder : public rclcpp::Node
{
public: 
    MocapForwarder() : 
		Node("mocap_forwarder"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)}
    {
        vehicle_mocap_pose_ned_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", qos_
        );
        mocap_pose_enu_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            MOCAP_TOPIC, qos_,
            std::bind(&MocapForwarder::mocap_pose_callback, this, std::placeholders::_1)
        );

        mocap_pose_enu_debug_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/offboard/mocap_orientation_enu_debug", qos_
        );
        mocap_px4_ned_debug_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/offboard/mocap_orientation_ned_debug", qos_
        );
	}

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_mocap_pose_ned_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_enu_subscription_;
    void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr ros2_msg);

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mocap_pose_enu_debug_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mocap_px4_ned_debug_publisher_;
    void publish_mocap_enu_euler_angles(const Eigen::Quaterniond &q);
    void publish_vehicle_mocap_ned_euler_angles(const Eigen::Quaterniond &q);
};

/**
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via mavlink to the vehicle autopilot filter to merge
 * @param ros2_msg A message with the pose of the vehicle expressed in ENU
 */
 void MocapForwarder::mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr ros2_msg) {
    px4_msgs::msg::VehicleOdometry px4_msg {};
    px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Convert ENU to NED
    Eigen::Vector3d ros2_enu_position = Eigen::Vector3d(
        ros2_msg->pose.position.x,
        ros2_msg->pose.position.y,
        ros2_msg->pose.position.z
    );
    Eigen::Vector3d px4_ned_position = transform_static_frame(ros2_enu_position, StaticTF::ENU_TO_NED);
    px4_msg.position[0] = px4_ned_position.x();
    px4_msg.position[1] = px4_ned_position.y();
    px4_msg.position[2] = px4_ned_position.z();

    // Convert quaternion from ROS2 to PX4 format
    Eigen::Quaterniond ros2_enu_orientation = Eigen::Quaterniond(
        ros2_msg->pose.orientation.w,
        ros2_msg->pose.orientation.x,
        ros2_msg->pose.orientation.y,
        ros2_msg->pose.orientation.z
    );
    // BASELINK_TO_AIRCRAFT    INPUT          ENU_TO_NED
    // ROTATION NED-ENU * q (ENU - FLU) * ROTATION FLU - FRD
    Eigen::Quaterniond intermediate_aircraft_orientation = transform_orientation(ros2_enu_orientation, StaticTF::BASELINK_TO_AIRCRAFT);
    Eigen::Quaterniond px4_ned_orientation = transform_orientation(intermediate_aircraft_orientation, StaticTF::ENU_TO_NED);
    px4_msg.q[0] = px4_ned_orientation.w();
    px4_msg.q[1] = px4_ned_orientation.x();
    px4_msg.q[2] = px4_ned_orientation.y();
    px4_msg.q[3] = px4_ned_orientation.z();

    // Mocap does not provide, so we set them to zero
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    px4_msg.velocity[0] = 0.0f;
    px4_msg.angular_velocity[0] = 0.0f;
    px4_msg.position_variance[0] = 0.0f;
    px4_msg.orientation_variance[0] = 0.0f;
    px4_msg.velocity_variance[0] = 0.0f;

    vehicle_mocap_pose_ned_publisher_->publish(px4_msg);

    publish_mocap_enu_euler_angles(ros2_enu_orientation);
    publish_vehicle_mocap_ned_euler_angles(px4_ned_orientation);
}

void MocapForwarder::publish_mocap_enu_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_degrees(q);

    geometry_msgs::msg::Vector3 msg;
    msg.x = euler.roll;
    msg.y = euler.pitch;
    msg.z = euler.yaw;
    mocap_pose_enu_debug_publisher_->publish(msg);
}

void MocapForwarder::publish_vehicle_mocap_ned_euler_angles(const Eigen::Quaterniond &q) {
    auto euler = one_degree_freedom::frame_transforms::quaternion_to_euler_degrees(q);

    geometry_msgs::msg::Vector3 msg;
    msg.x = euler.roll;
    msg.y = euler.pitch;
    msg.z = euler.yaw;
    mocap_px4_ned_debug_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting Mocap Forwarder node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapForwarder>());

	rclcpp::shutdown();
	return 0;
}
