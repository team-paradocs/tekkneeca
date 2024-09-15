#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <lbr_fri_msgs/msg/lbr_torque_command.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocityPublisher = nullptr;
rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher = nullptr;
rclcpp::Client<ocs2_msgs::srv::Reset>::SharedPtr mpcResetServiceClient = nullptr;
bool MpcReset = false;
sensor_msgs::msg::JointState last_joint_state_;

rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr testPublisher = nullptr;
rclcpp::TimerBase::SharedPtr timer_;

// double publish_frequency = 0.01; // 100 Hz, adjust as needed

// ocs2_msgs::msg::MpcObservation prepare_observation(const sensor_msgs::msg::JointState& msg)
// {
//     ocs2_msgs::msg::MpcObservation observation;
//     observation.state.value.resize(7); // Assuming 7 joints
//     observation.input.value.resize(7); // Assuming 7 inputs corresponding to 7 joints

//     for (size_t i = 0; i < 7; i++) {
//         observation.state.value[i] = msg.position[i];
//         observation.input.value[i] = msg.effort[i];
//     }

//     // Using the time from the JointState header for the current time
//     double curTime = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - 1713674280;
//     // double curTime = msg.header.stamp.nanosec * 1e-9;
//     observation.time = curTime;

//     // Assuming mode is always 0 for simplicity; adjust as needed for your application
//     observation.mode = 0;
//     return observation;
// }

// void publish_observation()
// {
//     // Publish observation
//     if (!last_joint_state_.position.empty()) {
//         auto observation = prepare_observation(last_joint_state_);
//         observationPublisher->publish(observation);
//         RCLCPP_INFO_STREAM(node->get_logger(), ">>> Observation is published at " << observation.time);
//     }
//     else {
//         RCLCPP_WARN(node->get_logger(), "No joint state message received yet.");
//     }
// }


// void joint_topic_callback(sensor_msgs::msg::JointState::SharedPtr msg)
// {
//     // RCLCPP_INFO(node->get_logger(), "From JointState I heard: %lf", msg->position[0]);

//     if (!MpcReset)
//     {
//         // MPC reset service client
//         std::string topicPrefix_ = "mobile_manipulator";
//         mpcResetServiceClient =
//             node->create_client<ocs2_msgs::srv::Reset>(topicPrefix_ + "_mpc_reset");

//         while (!mpcResetServiceClient->wait_for_service(std::chrono::seconds(5)) &&
//                 rclcpp::ok()) {
//             RCLCPP_ERROR_STREAM(node->get_logger(),
//                                 "Failed to call service to reset MPC, retrying...");
//         }

//         auto resetSrvRequest = std::make_shared<ocs2_msgs::srv::Reset::Request>();
//         resetSrvRequest->reset = static_cast<uint8_t>(true);

//         ocs2_msgs::msg::MpcTargetTrajectories targetTraj = ocs2_msgs::msg::MpcTargetTrajectories();

//         double curTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 - 1713674280;
//         // double curTime = msg->header.stamp.nanosec * 1e-9;
//         targetTraj.time_trajectory.resize(1);
//         targetTraj.time_trajectory[0] = curTime;

//         targetTraj.state_trajectory.resize(1);
//         // Assuming targetTraj.state_trajectory is a std::vector<MpcState>
//         for (auto& state : targetTraj.state_trajectory) {
//             state.value.resize(7);  // Resize the 'value' member of each MpcState
//             for (size_t i = 0; i < 7; i++) {
//                 state.value[i] = msg->position[i];
//             }
//         }
//         targetTraj.input_trajectory.resize(1);
//         for (auto& input : targetTraj.input_trajectory) {
//             input.value.resize(7);  // Resize the 'value' member of each MpcState
//             for (size_t i = 0; i < 7; i++) {
//                 input.value[i] = msg->effort[i];
//             }
//         }

//         testPublisher->publish(targetTraj);

//         resetSrvRequest->target_trajectories = targetTraj;
//         mpcResetServiceClient->async_send_request(resetSrvRequest);
//         RCLCPP_INFO_STREAM(node->get_logger(), "MPC node has been reset.");
//         MpcReset = true;
//     }
//     RCLCPP_INFO_STREAM(node->get_logger(), 
//         "### Current time " << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 - 1713674280);
//     last_joint_state_ = *msg; // Cache the latest joint state message
// }

void topic_callback(ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg)
{
    // RCLCPP_INFO(node->get_logger(), "From MPC I heard: %lf", msg->init_observation.state.value[0]);
    // curTime = msg->init_observation.time;
    // joint 0 torque
    // msg->input_trajectory[0]

    std_msgs::msg::Float64MultiArray velocity_command = std_msgs::msg::Float64MultiArray();
    velocity_command.data.resize(7);
    for (size_t i = 0; i < 7; i++) {
        velocity_command.data[i] = static_cast<double>(msg->input_trajectory[0].value[i]);
    }

    // Configure the layout of the array
    velocity_command.layout.dim.resize(1);  // Resize to 1 dimension
    velocity_command.layout.dim[0].label = "velocity";  // Label the dimension
    velocity_command.layout.dim[0].size = 7;  // There are 7 elements
    velocity_command.layout.dim[0].stride = 7;  // Total number of elements in the data

    // Optionally, set the offset if needed
    velocity_command.layout.data_offset = 0;

    RCLCPP_INFO_STREAM(node->get_logger(), ">>> New MPC policy starting at " << msg->init_observation.time);
    velocityPublisher->publish(velocity_command);
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
    // Get node parameters
    // std::string taskFile = node->get_parameter("taskFile").as_string();
    // std::string libFolder = node->get_parameter("libFolder").as_string();
    // std::string urdfFile = node->get_parameter("urdfFile").as_string();
    // std::cerr << "Loading task file: " << taskFile << std::endl;
    // std::cerr << "Loading library folder: " << libFolder << std::endl;
    // std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    // Robot Interface
    //   mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder,
    //                                                            urdfFile);

    // MRT
    // MRT_ROS_Interface mrt(robotName);
    // mrt.initRollout(&interface.getRollout());
    // mrt.launchNodes(node);

    // Visualization
    // auto dummyVisualization =
    //     std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(
    //         node, interface);

    // Dummy MRT
    // MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_,
    //                         interface.mpcSettings().mpcDesiredFrequency_);
    // dummy.subscribeObservers({dummyVisualization});

    // initial state
    // SystemObservation initObservation;
    // initObservation.state = interface.getInitialState();
    // initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
    // initObservation.time = 0.0;

    // initial command
    //   vector_t initTarget(7);
    //   initTarget.head(3) << 1, 0, 1;
    //   initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    //   const vector_t zeroInput =
    //       vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
    //   const TargetTrajectories initTargetTrajectories({initObservation.time},
    //                                                   {initTarget}, {zeroInput});

    // Run dummy (loops while ros is ok)
    //   dummy.run(initObservation, initTargetTrajectories);

    auto mpcSubscription = node->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
    "mobile_manipulator_mpc_policy", 10, topic_callback);

    // auto stateSubscription = node->create_subscription<sensor_msgs::msg::JointState>("lbr/joint_states", 10, joint_topic_callback);

    velocityPublisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("lbr/forward_velocity_controller/commands", 10);
    // observationPublisher = node->create_publisher<ocs2_msgs::msg::MpcObservation>("mobile_manipulator_mpc_observation", 10);

    // testPublisher = node->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>("test_out", 10);

    // rclcpp::Clock clock(RCL_ROS_TIME);

    // while (rclcpp::ok()) {
    //     rclcpp::Time now = clock.now();
    //     curTime = now.seconds();
    //     // RCLCPP_INFO(node->get_logger(), "Current time: %lf", curTime);
    // }

    // timer_ = node->create_wall_timer(
    // std::chrono::milliseconds(int(1000 * publish_frequency)), publish_observation);

    rclcpp::spin(node);
    rclcpp::shutdown();
    // Successful exit
    return 0;
}