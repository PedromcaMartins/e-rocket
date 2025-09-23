#pragma once
#include <rclcpp/rclcpp.hpp>
#include <one_degree_freedom/constants.hpp>
#include <one_degree_freedom/frame_transforms.h>

#include <one_degree_freedom/controller/state.hpp>
#include <one_degree_freedom/controller/setpoint.hpp>
#include <one_degree_freedom/controller/allocator.hpp>

using namespace one_degree_freedom::frame_transforms;

class GenericController
{
public:
    GenericController(
        rclcpp::Node* node, 
        rclcpp::QoS qos, 
        std::shared_ptr<StateAggregator> state_aggregator, 
        std::shared_ptr<SetpointAggregator> setpoint_aggregator,
        double controller_freq
    ): 
        state_aggregator_(state_aggregator),
        setpoint_aggregator_(setpoint_aggregator),
        dt_{1.0 / controller_freq}
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

    double dt_;
    std::pair<ServoAllocatorInput, MotorAllocatorInput> output_;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Logger logger_ = rclcpp::get_logger("generic_controller");

    void publish_debug() {
        RCLCPP_INFO(logger_, "Generic controller debug info");
    }
};
