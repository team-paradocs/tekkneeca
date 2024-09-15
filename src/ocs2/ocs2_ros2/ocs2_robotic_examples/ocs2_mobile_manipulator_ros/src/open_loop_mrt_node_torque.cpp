#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <lbr_fri_msgs/msg/lbr_torque_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_msgs/srv/reset.hpp>

// #include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"
// #include <ocs2_core/control/FeedforwardController.h>
// #include <ocs2_core/control/LinearController.h>

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "rclcpp/rclcpp.hpp"

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<lbr_fri_msgs::msg::LBRTorqueCommand>::SharedPtr torquePublisher = nullptr;
rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher = nullptr;

// void joint_topic_callback(sensor_msgs::msg::JointState::SharedPtr msg)
// {
//     ocs2_msgs::msg::MpcObservation observation;
//     observation.state.value.resize(7); // Assuming 7 joints
//     observation.input.value.resize(7); // Assuming 7 inputs corresponding to 7 joints

//     for (size_t i = 0; i < 7; i++) {
//         observation.state.value[i] = msg->position[i];
//         observation.input.value[i] = msg->effort[i];
//     }

//     // Using the time from the JointState header for the current time
//     double curTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
//     observation.time = curTime;

//     // Assuming mode is always 0 for simplicity; adjust as needed for your application
//     observation.mode = 0;

//     // Publish the observation
//     observationPublisher->publish(observation);
// }

void topic_callback(ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg)
{
    lbr_fri_msgs::msg::LBRTorqueCommand torque_command = lbr_fri_msgs::msg::LBRTorqueCommand();
    for (size_t i = 0; i < 7; i++) {
        torque_command.joint_position[i] = static_cast<double>(msg->state_trajectory[0].value[i]);
        torque_command.torque[i] = static_cast<double>(msg->input_trajectory[0].value[i]);
    }
    torquePublisher->publish(torque_command);
}

int main(int argc, char** argv) {
    const std::string robotName = "mobile_manipulator";

    // Initialize ros node
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared(
        robotName + "_mrt",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    auto mpcSubscription = node->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
    "mobile_manipulator_mpc_policy", 10, topic_callback);

    // auto stateSubscription = node->create_subscription<sensor_msgs::msg::JointState>("lbr/joint_states", 10, joint_topic_callback);

    torquePublisher = node->create_publisher<lbr_fri_msgs::msg::LBRTorqueCommand>("lbr/command/torque", 10);
    // observationPublisher = node->create_publisher<ocs2_msgs::msg::MpcObservation>("mobile_manipulator_mpc_observation", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    // Successful exit
    return 0;
}