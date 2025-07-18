#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <one_degree_freedom/frame_transforms.h>
#include <one_degree_freedom/constants.hpp>

#include <eigen3/Eigen/Geometry>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace one_degree_freedom::frame_transforms;
using namespace one_degree_freedom::constants::mocap_forwarder;

/**
 * @brief PX4 ROS2 Communication Node is responsible for sending and receiving commands to and from the PX4. 
 */
class MocapForwarderTest : public rclcpp::Node
{
public: 
    MocapForwarderTest() : 
		Node("flight_mode_test"),
		qos_profile_{rmw_qos_profile_sensor_data},
		qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},
        mocap_publisher_{this->create_publisher<geometry_msgs::msg::PoseStamped>(
            MOCAP_TOPIC, qos_
        )}
    {
        test_timer_ = this->create_wall_timer(100ms,
            [this]() {
                geometry_msgs::msg::PoseStamped mocap_pose_enu_msg;
                mocap_pose_enu_msg.header.stamp = this->get_clock()->now();
                mocap_pose_enu_msg.header.frame_id = "mocap_frame";
            
                // Set a dummy pose for testing
                double time = this->get_clock()->now().seconds();
                mocap_pose_enu_msg.pose.position.x = 5. * sin(time);
                mocap_pose_enu_msg.pose.position.y = 5. * cos(time);
                mocap_pose_enu_msg.pose.position.z = 5. * sin(2 * time);
            
                auto q = euler_radians_to_quaternion(
                    EulerAngle{
                        0.1 * sin(time),
                        0.1 * cos(time),
                        0.1 * sin(2 * time)
                    }
                );
            
                mocap_pose_enu_msg.pose.orientation.w = q.w();
                mocap_pose_enu_msg.pose.orientation.x = q.x();
                mocap_pose_enu_msg.pose.orientation.y = q.y();
                mocap_pose_enu_msg.pose.orientation.z = q.z();
            
                mocap_publisher_->publish(mocap_pose_enu_msg);
            }
		);
    }

private:
	rmw_qos_profile_t qos_profile_;
	rclcpp::QoS qos_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_publisher_;

	rclcpp::TimerBase::SharedPtr test_timer_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 ROS2 Message Mapping Test node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapForwarderTest>());

	rclcpp::shutdown();
	return 0;
}
