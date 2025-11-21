#pragma once
#include <rclcpp/rclcpp.hpp>
#include <erocket/constants.hpp>
#include <px4_msgs/msg/actuator_armed.hpp>

using namespace erocket::constants::emergency;
using namespace px4_msgs::msg;

class EmergencySwitch {
public:
    EmergencySwitch(
        rclcpp::Node* node,
        rclcpp::QoS qos
    ) :
        actuator_armed_sub_{node->create_subscription<ActuatorArmed>(
            EMERGENCY_ACTUATOR_ARMED,
            qos,
            [this](const ActuatorArmed::SharedPtr msg) {
                if (msg->manual_lockdown) {
                    emergency_switch_on_ = true;
                    RCLCPP_WARN(logger_, "Emergency switch is ON!");
                } else {
                    RCLCPP_INFO(logger_, "Emergency switch is OFF.");
                }
            }
        )}
    {
    }

    bool emergency_switch_on() {
        return emergency_switch_on_;
    }

private:
    rclcpp::Logger logger_{rclcpp::get_logger("EmergencySwitch")};
    rclcpp::Subscription<ActuatorArmed>::SharedPtr actuator_armed_sub_;
    bool emergency_switch_on_{false};
};
