/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

class MoveGroupNode : public rclcpp::Node 
{
public:
  MoveGroupNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .allow_undeclared_parameters(true))
    : Node("mgi_node", options)
  {
    RCLCPP_INFO(LOGGER, "Auto declare %d", Node::get_node_options().automatically_declare_parameters_from_overrides());
    RCLCPP_INFO(LOGGER, "Allow undeclare %d", Node::get_node_options().allow_undeclared_parameters());
  }

  std::shared_ptr<MoveGroupNode> shared_from_this()
  {
    return std::static_pointer_cast<MoveGroupNode>(Node::shared_from_this());
  }

  void initialize()
  {

    static const std::string PLANNING_GROUP = this->get_parameter("planning_group").as_string();
    moveit::planning_interface::MoveGroupInterface::Options move_group_interface_options(PLANNING_GROUP, "robot_description", "lbr");

    RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface");
    // from static_obstacles.cpp
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), 
      move_group_interface_options,
      std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration(0, 0)
    );
    RCLCPP_INFO(LOGGER, "Created");

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_interface_->getJointModelGroupNames().begin(), move_group_interface_->getJointModelGroupNames().end(),
      std::ostream_iterator<std::string>(std::cout, ", "));

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(move_group_interface_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description", std::shared_ptr<tf2_ros::Buffer>(), "planning_scene_monitor");

    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    // Visualization
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(),
      "lbr/link_0",
      "rviz_visual_tools",
       move_group_interface_->getRobotModel());
    visual_tools_->deleteAllMarkers();

    start_drilling_publisher_ = this->create_publisher<std_msgs::msg::String>("drill_commands", 10);
    
    joint_trajectory_action_client_ = 
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "joint_trajectory_controller/follow_joint_trajectory");
    // Wait until the action server is available
    while (!joint_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 5000 /* ms */, 
        "Waiting for FollowJointTrajectory action server to be available...");
    }
    RCLCPP_INFO(LOGGER, "FollowJointTrajectory Action server available.");

    goal_options_ = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    goal_options_.goal_response_callback = 
      [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle) {
        // auto goal_handle = goalHandle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(LOGGER, "Goal was rejected by the joint trajectory controller action server.");
        } else {
            // RCLCPP_INFO(LOGGER, "Goal accepted by the joint trajectory controller action server, executing...");
        }
      };        

    goal_options_.feedback_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr /*unused*/,
                                            const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> /*unused*/) {
        // RCLCPP_INFO(LOGGER, "Received feedback: %f", feedback->desired.positions[0]);  // Access feedback values
    };

    goal_options_.result_callback = [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // RCLCPP_INFO(LOGGER, "Goal to the joint trajectory controller action server succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                // RCLCPP_ERROR(LOGGER, "Goal to the joint trajectory controller action server aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                // RCLCPP_ERROR(LOGGER, "Goal to the joint trajectory controller action server canceled.");
                break;
            default:
                // RCLCPP_ERROR(LOGGER, "Goal to the joint trajectory controller action server results unknown result code.");
                break;
        }
    };


    // Initialize tracking goal subscriber
    tracking_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "tracked_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {

          // RCLCPP_INFO(LOGGER, "Tracking goal received");

          // always cache the tracking goal
          auto temp_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
          temp_pose->pose = msg->pose;

          // fix pilz bug
          temp_pose->header.frame_id = "world";
          hybrid_planning_goal_handle_ = std::move(temp_pose);

          if (planner_state_ == 0 || planner_state_ == 2 || planner_state_ == 3) {
            // not planning
          }
          else if (planner_state_ == 1)
          {
            planXexecute(*(hybrid_planning_goal_handle_.get()));
          }
          else
          {
            RCLCPP_ERROR(LOGGER, "Invalid planner state: %d", planner_state_.load());
          }
        }
    );

    drilling_goal_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "surgical_drill_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) {
          if (drill_state_ == 0) {
            auto drill_pose = msg->poses[0];
            auto drill_pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
            drill_pose_stamped->pose = drill_pose;
            
            // fix pilz bug
            drill_pose_stamped->header.frame_id = "world";
            drill_pose_goal_handle_ = std::move(drill_pose_stamped);
          }
        }
    );

    // Initialize planning state subscriber
    planning_state_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/lbr/plan_flag", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int32::ConstSharedPtr& msg) {
          RCLCPP_INFO(LOGGER, "Planning flag received: %d", msg->data);

          planner_state_ = msg->data;
          if (planner_state_ == 1) {
            // planning + execution mode for now
            planXexecute(*(hybrid_planning_goal_handle_.get()));
          }
          else if (planner_state_ == 0) {
            // stop planning stop execution
            joint_trajectory_action_client_->async_cancel_all_goals();
          }
          else if (planner_state_ == 2) {

            // start drill motion
            // if (drill_state_ == 0) {
            //   drill_state_ = 1;
            //   drillMotion();
            // }
          }
          else if (msg->data == 3) 
          {
            // hard reset and go back to home

            // TODO: cancel execution first
            drill_state_ = 0;
            drillCmd(false);


          }
          else
            RCLCPP_ERROR(LOGGER, "Invalid planning flag received: %d", msg->data);
        }
    );
  }

  geometry_msgs::msg::PoseStamped computeOffsetPose(const geometry_msgs::msg::PoseStamped& targetPose, float offset = 0.02)
  {
    float x_hard_coded_offset = 0.00;
    float y_hard_coded_offset = 0.00;

    // Calculate the final drill position
    // Convert the quaternion to a rotation matrix
    Eigen::Quaterniond quat(targetPose.pose.orientation.w,
                            targetPose.pose.orientation.x,
                            targetPose.pose.orientation.y,
                            targetPose.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Create a vector representing the direction to move in
    Eigen::Vector3d direction(0, 0, offset);

    // Multiply the rotation matrix by the direction vector
    Eigen::Vector3d result = rotation_matrix * direction;

    // Add the result to the original position to get the new position
    geometry_msgs::msg::PoseStamped newPose;
    newPose.pose.position.x = targetPose.pose.position.x + result(0) + x_hard_coded_offset;
    newPose.pose.position.y = targetPose.pose.position.y + result(1) + y_hard_coded_offset;
    newPose.pose.position.z = targetPose.pose.position.z + result(2);

    // The orientation remains the same
    newPose.pose.orientation = targetPose.pose.orientation;

    newPose.header.frame_id = targetPose.header.frame_id;
    newPose.header.stamp = targetPose.header.stamp;

    return newPose;
  }



  void drillMotion()
  {
    int stage = drill_state_.load();

    RCLCPP_INFO(LOGGER, "In function drillMotion, drill_state_: %d", stage);

    if (drill_pose_goal_handle_ == nullptr)
    {
      RCLCPP_ERROR(LOGGER, "drill_pose_goal_handle_ is nullptr");
      drill_state_ = 0;
      return;
    }

    geometry_msgs::msg::PoseStamped preDrillPose = computeOffsetPose(*(drill_pose_goal_handle_.get()), -0.05);

    geometry_msgs::msg::PoseStamped startDrillPose = computeOffsetPose(*(drill_pose_goal_handle_.get()), -0.015);

    geometry_msgs::msg::PoseStamped touchPose = computeOffsetPose(*(drill_pose_goal_handle_.get()), -0.001);

    geometry_msgs::msg::PoseStamped endPose = computeOffsetPose(*(drill_pose_goal_handle_.get()), 0.025);

    geometry_msgs::msg::PoseStamped offDrillPose = computeOffsetPose(*(drill_pose_goal_handle_.get()), -0.015);

    std::vector<double> joint_values = { 0.0, 0.0, 0.0, 1.57, 0.0, -1.57, 0.0};
    moveit::core::RobotState temp_state(planning_scene_monitor_->getRobotModel());
    temp_state.setJointGroupPositions(joint_model_group_.get(), joint_values);

    planXexecute(preDrillPose);



  }

  void drillCmd(bool start)
  {
    std_msgs::msg::String msg;
    if (start) {
      msg.data = "d";
    } else {
      msg.data = "s";
    }
    start_drilling_publisher_->publish(msg);
  }

  void planXexecute(const geometry_msgs::msg::PoseStamped& goalPose)
  {
    // geometry_msgs::msg::Pose target_pose1;
    // target_pose1.position.x = -0.4;
    // target_pose1.position.y = 0.2;
    // target_pose1.position.z = 0.36;
    // target_pose1.orientation.x = 0.6411;
    // target_pose1.orientation.y = 0.7623;
    // target_pose1.orientation.z = -0.0516;
    // target_pose1.orientation.w = 0.0724;

    auto planning_scene = planning_scene_monitor_->getPlanningScene();
    move_group_interface_->setStartStateToCurrentState();
    auto tmp_state = planning_scene->getCurrentStateNonConst();

    // std::shared_ptr<moveit::core::RobotState> tmp_state = move_group_interface_->getCurrentState(10);
    bool ik_success = tmp_state.setFromIK(joint_model_group_.get(), goalPose.pose);
    RCLCPP_INFO(LOGGER, "IK success: %d", ik_success);
    std::vector<double> joint_values;
    tmp_state.copyJointGroupPositions(joint_model_group_.get(), joint_values);
    // for (size_t i = 0; i < joint_values.size(); ++i)
    // {
    //   RCLCPP_WARN(LOGGER, "Joint %d: %f", i+1, joint_values[i]);
    // }

    move_group_interface_->setJointValueTarget(joint_values);

    // move_group_interface_->setPoseTarget(target_pose1, "link_tool");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Planning success: %d", success);
    
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    // visual_tools_->publishAxisLabeled(target_pose1, "pose1");

    const std::string ee_link_name = "link_tool"; // Replace with your end-effector link name
    const moveit::core::LinkModel* ee_link = planning_scene_monitor_->getRobotModel()->getLinkModel(ee_link_name);
    visual_tools_->publishTrajectoryLine(my_plan.trajectory_, ee_link, joint_model_group_.get());
    visual_tools_->trigger();

    if (planner_state_ != 0) {
      // move_group_interface_->execute(my_plan);

      auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

      // Extract the JointTrajectory part
      goal_msg.trajectory = my_plan.trajectory_.joint_trajectory;
      joint_trajectory_action_client_->async_send_goal(goal_msg, goal_options_);

    }
  }

private:
  const rclcpp::Logger LOGGER = rclcpp::get_logger("mgi_node");

  std::shared_ptr<const moveit::core::JointModelGroup> joint_model_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options_;

  // Start drilling publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_drilling_publisher_;

  // Tracking goal subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_goal_sub_;

  // Drilling goal subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drilling_goal_sub_;

  // Planning state subscriber
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr planning_state_sub_;

  // Flag that indicates hybrid planning has been canceled
  std::atomic<int> planner_state_;

  // 0: not drilling, 1: predrill, 2: finished drill
  std::atomic<int> drill_state_;

  std::shared_ptr<geometry_msgs::msg::PoseStamped> hybrid_planning_goal_handle_;

  std::shared_ptr<geometry_msgs::msg::PoseStamped> drill_pose_goal_handle_;

  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupNode>();
  // Call initialize after creating the shared pointer instance
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}