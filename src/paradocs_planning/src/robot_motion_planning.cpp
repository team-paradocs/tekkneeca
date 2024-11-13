/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <std_msgs/msg/int32.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

class MoveItPlanningNode : public rclcpp::Node 
{
public:
  MoveItPlanningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .allow_undeclared_parameters(true))
    : Node("robot_motion_planning", options)
  {
    RCLCPP_INFO(LOGGER, "Auto declare %d", Node::get_node_options().automatically_declare_parameters_from_overrides());
    RCLCPP_INFO(LOGGER, "Allow undeclare %d", Node::get_node_options().allow_undeclared_parameters());
  }


  std::shared_ptr<MoveItPlanningNode> shared_from_this()
  {
    return std::static_pointer_cast<MoveItPlanningNode>(Node::shared_from_this());
  }

  void initialize()
  {

    // Initialize MoveItCpp API
    moveit_cpp::MoveItCpp::Options moveit_cpp_options(this->shared_from_this());
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this(), moveit_cpp_options);

    planning_group_ = this->get_parameter("planning_group").as_string();
    
    robot_model_ = moveit_cpp_->getRobotModel();
    goal_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(goal_state_->getJointModelGroup(planning_group_));
    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_);

    // Initialize tracking goal subscriber
    tracking_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "tracked_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          if (planner_state_ == 0) {
            // not planning
          }
          else if (planner_state_ == 1)
          {
            planMotionXexecutePlan(msg);
          } 
          else if (planner_state_ == 2)
          {
            if (!dill_started_) {
              dill_started_ = true;
              drillingMotion(msg);
            }
          }
          else
          {
            RCLCPP_ERROR(LOGGER, "Invalid planner state: %d", planner_state_.load());
          }
          
        }
    );

    // Initialize planning state subscriber
    planning_state_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/lbr/plan_flag", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int32::ConstSharedPtr& msg) {
          RCLCPP_INFO(LOGGER, "Planning flag received: %d", msg->data);
          if (msg->data == 1) {
            planner_state_ = 1;
          }
          else if (msg->data == 0) {
            planner_state_ = 0;
            joint_trajectory_action_client_->async_cancel_all_goals();
          }
          else if (msg->data == 2) {
            planner_state_ = 2;
          }
          else
            RCLCPP_ERROR(LOGGER, "Invalid planning flag received: %d", msg->data);
        }
    );

    // Local solution publisher is defined by the local constraint solver plugin
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

  }

  void planMotionXexecutePlan(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goalPose)
  {

    // RCLCPP_INFO(LOGGER, "Goal Pose Position: x: %f, y: %f, z: %f", goalPose->pose.position.x, goalPose->pose.position.y,
    //             goalPose->pose.position.z);

    // RCLCPP_INFO(LOGGER, "Goal Pose Orientation: x: %f, y: %f, z: %f, w: %f", goalPose->pose.orientation.x,
    //             goalPose->pose.orientation.y, goalPose->pose.orientation.z, goalPose->pose.orientation.w);


    // Set parameters required by the planning component
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
    plan_params.planner_id = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planner_id").as_string();
    plan_params.planning_pipeline = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_pipeline").as_string();
    plan_params.planning_attempts = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_attempts").as_int();
    plan_params.planning_time = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_time").as_double();
    plan_params.max_velocity_scaling_factor =
        this->get_parameter(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor").as_double();
    plan_params.max_acceleration_scaling_factor =
        this->get_parameter(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor").as_double();

    geometry_msgs::msg::PoseStamped mutableGoalPose = *goalPose;
    mutableGoalPose.header.frame_id = "world";
    // update planning scene with current state
    moveit_cpp_->getPlanningSceneMonitor()->updateSceneWithCurrentState();
    // Set start state to current state
    planning_component_->setStartStateToCurrentState();
    moveit_msgs::msg::Constraints goal_constraints =
        kinematic_constraints::constructGoalConstraints("link_tool", mutableGoalPose);
    planning_component_->setGoal({goal_constraints});

    // Plan motion
    auto plan_solution = planning_component_->plan(plan_params);
    if (plan_solution.error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Planning succeeded");
      if (!planner_state_==0)
      {
        // execute the global plan
        // RCLCPP_INFO(LOGGER, "Executing the plan");
        // true/false: isblocking or not
        // planning_component_->execute(false);

        // Local solution publisher is defined by the local constraint solver plugin
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
        time_parameterization_.computeTimeStamps(*plan_solution.trajectory, 
          plan_params.max_velocity_scaling_factor,
          plan_params.max_acceleration_scaling_factor);

        plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);

        // Extract the JointTrajectory part
        goal_msg.trajectory = robot_trajectory_msg.joint_trajectory;
        joint_trajectory_action_client_->async_send_goal(goal_msg, goal_options_);

      }
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  }

  geometry_msgs::msg::PoseStamped computeOffsetPose(const geometry_msgs::msg::PoseStamped targtPose, float offset = 0.02)
  {
    float x_hard_coded_offset = 0.00;
    float y_hard_coded_offset = 0.00;

    // Calculate the final drill position
    // Convert the quaternion to a rotation matrix
    Eigen::Quaterniond quat(targtPose.pose.orientation.w,
                            targtPose.pose.orientation.x,
                            targtPose.pose.orientation.y,
                            targtPose.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Create a vector representing the direction to move in
    Eigen::Vector3d direction(0, 0, offset);

    // Multiply the rotation matrix by the direction vector
    Eigen::Vector3d result = rotation_matrix * direction;

    // Add the result to the original position to get the new position
    geometry_msgs::msg::PoseStamped newPose;
    newPose.pose.position.x = targtPose.pose.position.x + result(0) + x_hard_coded_offset;
    newPose.pose.position.y = targtPose.pose.position.y + result(1) + y_hard_coded_offset;
    newPose.pose.position.z = targtPose.pose.position.z + result(2);

    // The orientation remains the same
    newPose.pose.orientation = targtPose.pose.orientation;

    newPose.header.frame_id = targtPose.header.frame_id;
    newPose.header.stamp = targtPose.header.stamp;

    return newPose;
  }

  void drillingMotion(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goalPose)
  {
    RCLCPP_INFO(LOGGER, "Drilling motion");

    geometry_msgs::msg::PoseStamped touchPose = computeOffsetPose(*(goalPose.get()), -0.001);

    // cartesian planning parameters
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory drillTrajectory;
    std::vector<moveit_msgs::msg::Constraints> drillWayPointConstraints;
    moveit_msgs::msg::Constraints touch_constraints =
      kinematic_constraints::constructGoalConstraints("link_tool", touchPose);

    drillWayPointConstraints.push_back(touch_constraints);

    int drillPoints = 25;

    geometry_msgs::msg::PoseStamped endPose = computeOffsetPose(*(goalPose.get()), 0.015);
    // Interpolate drill_points number of points between start and end pose
    std::stack<moveit_msgs::msg::Constraints> interpolatedConstraintsStack;

    //drill in
    for (int i = 0; i <= drillPoints; i++) {
      geometry_msgs::msg::PoseStamped interpolatedPose;
      float ratio = static_cast<float>(i) / drillPoints;
      // Interpolate position
      interpolatedPose.pose.position.x = touchPose.pose.position.x + ratio * (endPose.pose.position.x - touchPose.pose.position.x);
      interpolatedPose.pose.position.y = touchPose.pose.position.y + ratio * (endPose.pose.position.y - touchPose.pose.position.y);
      interpolatedPose.pose.position.z = touchPose.pose.position.z + ratio * (endPose.pose.position.z - touchPose.pose.position.z);
      // Orientation remains the same
      interpolatedPose.pose.orientation = touchPose.pose.orientation;
      // Copy header
      interpolatedPose.header.frame_id = touchPose.header.frame_id;
      interpolatedPose.header.stamp = touchPose.header.stamp;

      moveit_msgs::msg::Constraints interpolatedConstraints =
        kinematic_constraints::constructGoalConstraints("link_tool", interpolatedPose);

      interpolatedConstraintsStack.push(interpolatedConstraints);
      drillWayPointConstraints.push_back(interpolatedConstraints);
    }

    //drill out 
    while (!interpolatedConstraintsStack.empty()) {
      drillWayPointConstraints.push_back(interpolatedConstraintsStack.top());
      interpolatedConstraintsStack.pop();
    }

    // Set parameters required by the planning component
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
    plan_params.planner_id = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planner_id").as_string();
    plan_params.planning_pipeline = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_pipeline").as_string();
    plan_params.planning_attempts = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_attempts").as_int();
    plan_params.planning_time = this->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_time").as_double();
    plan_params.max_velocity_scaling_factor =
        this->get_parameter(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor").as_double();
    plan_params.max_acceleration_scaling_factor =
        this->get_parameter(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor").as_double();


    for (auto& constraints : drillWayPointConstraints)
    {
      // RCLCPP_INFO(LOGGER, "Drill waypoint constraints: %s", constraint.name.c_str());

      // update planning scene with current state
        moveit_cpp_->getPlanningSceneMonitor()->updateSceneWithCurrentState();
        // Set start state to current state
        planning_component_->setStartStateToCurrentState();
        planning_component_->setGoal({constraints});

        // Plan motion
        auto plan_solution = planning_component_->plan(plan_params);
        if (plan_solution.error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          // RCLCPP_INFO(LOGGER, "Planning succeeded");
          if (!planner_state_==0)
          {
            // execute the global plan
            // RCLCPP_INFO(LOGGER, "Executing the plan");
            // true/false: isblocking or not
            planning_component_->execute(true);
            // // Local solution publisher is defined by the local constraint solver plugin
            // auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

            // moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
            // time_parameterization_.computeTimeStamps(*plan_solution.trajectory, 
            //   plan_params.max_velocity_scaling_factor,
            //   plan_params.max_acceleration_scaling_factor);

            // plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);

            // // Extract the JointTrajectory part
            // goal_msg.trajectory = robot_trajectory_msg.joint_trajectory;

            // joint_trajectory_action_client_->async_send_goal(goal_msg, goal_options_);
          }
        }
        else
        {
          RCLCPP_ERROR(LOGGER, "Planning failed");
        }
    }

  }





private:
  const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_motion_planning");
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  const std::string PLANNING_SCENE_MONITOR_NS = "planning_scene_monitor_options.";
  const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
  const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
  const std::string UNDEFINED = "<undefined>";
  // rclcpp::Service<icdt_interfaces::srv::MotionPlanning>::SharedPtr MotionPlanningService;

  // Tracking goal subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_goal_sub_;

  // Planning state subscriber
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr planning_state_sub_;

  std::shared_ptr<const moveit::core::JointModelGroup> joint_model_group_;

  // Robot model
  std::shared_ptr<const moveit::core::RobotModel> robot_model_;

  // Planning group
  std::string planning_group_;

  // Goal from IK calculation
  std::shared_ptr<moveit::core::RobotState> goal_state_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;

  // Flag to avoid publishing the global plan
  std::atomic<int> planner_state_{0};

  // Local solution action client
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options_;

  trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization_;

  std::atomic<bool> dill_started_ = false;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItPlanningNode>();

  // Call initialize after creating the shared pointer instance
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}