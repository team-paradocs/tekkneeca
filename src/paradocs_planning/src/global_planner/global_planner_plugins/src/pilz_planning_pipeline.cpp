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

#include <moveit/global_planner/pilz_planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>


namespace
{
  const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
}

namespace moveit::hybrid_planning
{
  const std::string PLANNING_SCENE_MONITOR_NS = "planning_scene_monitor_options.";
  const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
  const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
  const std::string UNDEFINED = "<undefined>";

  bool PilzPlanningPipeline::initialize(const rclcpp::Node::SharedPtr &node)
  {

    node_ptr_ = node;

    moveit_cpp::MoveItCpp::Options moveit_cpp_options(node);
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);

    // Get the robot model for IK
    robot_model_ = moveit_cpp_->getRobotModel();
    goal_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(
        goal_state_->getJointModelGroup("arm"));
   
    rclcpp::sleep_for(std::chrono::seconds(1));

    return true;
  }

  bool PilzPlanningPipeline::reset() noexcept
  {
    // Do Nothing
    return true;
  }

  moveit_msgs::msg::MotionPlanResponse PilzPlanningPipeline::plan(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
  {

    moveit_msgs::msg::MotionPlanResponse response;

    if ((global_goal_handle->get_goal())->motion_sequence.items.empty())
    {
      RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with no items. At least one is needed.");
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return response;
    }

    // Process goal
    if ((global_goal_handle->get_goal())->motion_sequence.items.size() > 1)
    {
      RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with more than one item but the "
                          "'moveit_planning_pipeline' plugin only accepts one item. Just using the first item as global "
                          "planning goal!");
    }
    auto motion_plan_req = (global_goal_handle->get_goal())->motion_sequence.items[0].req;
    auto group_name = (global_goal_handle->get_goal())->planning_group;

    // Set parameters required by the planning component
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
    plan_params.planner_id = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planner_id").as_string();
    RCLCPP_INFO(LOGGER, "Planner ID: %s", plan_params.planner_id.c_str());
    plan_params.planning_pipeline = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_pipeline").as_string();
    plan_params.planning_attempts = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_attempts").as_int();
    plan_params.planning_time = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_time").as_double();
    RCLCPP_INFO(LOGGER, "Planner time: %f", plan_params.planning_time);

    plan_params.max_velocity_scaling_factor = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor").as_double();
    plan_params.max_acceleration_scaling_factor = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor").as_double();

    // Create planning component
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(group_name, moveit_cpp_);

    // create the moveit interface, and update the planning scene
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitor();
    auto planning_scene = planning_scene_monitor->getPlanningScene();

    // Set start state to current state
    planning_components->setStartStateToCurrentState();
    auto current_state = planning_scene->getCurrentStateNonConst();
    const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());

    // Plan the trajectory using pilz

    // Fix Pilz bug
    motion_plan_req.goal_constraints[0].position_constraints[0].header.frame_id = "world";
    motion_plan_req.goal_constraints[0].orientation_constraints[0].header.frame_id = "world";

    planning_components->setGoal(motion_plan_req.goal_constraints);

    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // Use system time (or RCL_ROS_TIME for simulation time)

    rclcpp::Time last_call_time = clock.now();

    // Plan motion
    auto plan_solution = planning_components->plan(plan_params);
    if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      response.error_code = plan_solution.error_code;
      return response;
    }

    // Get current time
    rclcpp::Time current_time = clock.now();
    
    // Calculate the time difference in seconds
    double time_difference = (current_time - last_call_time).seconds();
    RCLCPP_WARN(LOGGER, "Planning time since last call: %.6f seconds", time_difference);

    // Transform solution into MotionPlanResponse and publish it
    response.trajectory_start = plan_solution.start_state;
    response.group_name = group_name;
    plan_solution.trajectory->getRobotTrajectoryMsg(response.trajectory);
    response.error_code = plan_solution.error_code;

    return response;
  }
  
} // namespace moveit::hybrid_planning


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::PilzPlanningPipeline, moveit::hybrid_planning::GlobalPlannerInterface);
