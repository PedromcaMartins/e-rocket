#pragma once
#include <rclcpp/rclcpp.hpp>
#include <erocket/constants.hpp>
#include <erocket/frame_transforms.h>

#include <erocket/controller/state.hpp>
#include <erocket/controller/setpoint.hpp>
#include <erocket/controller/allocator.hpp>
#include <erocket/vehicle_constants.hpp>
#include <erocket/msg/generic_controller_debug.hpp>

using namespace erocket::frame_transforms;
using namespace erocket::msg;

class GenericController
{
public:
    GenericController(
        rclcpp::Node* node, 
        rclcpp::QoS qos, 
        std::shared_ptr<StateAggregator> state_aggregator, 
        std::shared_ptr<SetpointAggregator> setpoint_aggregator,
        std::shared_ptr<VehicleConstants> vehicle_constants,
        double controller_period
    ): 
        state_aggregator_(state_aggregator),
        setpoint_aggregator_(setpoint_aggregator),
        vehicle_constants_(vehicle_constants),
        dt_{controller_period},
        debug_publisher_{node->create_publisher<GenericControllerDebug>("generic_controller/debug", qos)}
    { }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    std::pair<ServoAllocatorInput, MotorAllocatorInput> compute() {
        auto state = state_aggregator_->get_state();
        auto setpoint = setpoint_aggregator_->get_attitude_setpoint();

        // execute controller

        publish_debug();
        return output_;
    }

private:
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    std::shared_ptr<VehicleConstants> vehicle_constants_;

    rclcpp::Publisher<GenericControllerDebug>::SharedPtr debug_publisher_;

    double dt_;
    std::pair<ServoAllocatorInput, MotorAllocatorInput> output_;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Logger logger_ = rclcpp::get_logger("generic_controller");

    void publish_debug() {
        auto message = GenericControllerDebug();

        // debug me! :D

        debug_publisher_->publish(message);
    }
};
