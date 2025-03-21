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

#include <moveit/global_planner/moveit_planning_pipeline.h>
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

  bool MoveItPlanningPipeline::initialize(const rclcpp::Node::SharedPtr &node)
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

  bool MoveItPlanningPipeline::reset() noexcept
  {
    // Do Nothing
    return true;
  }

  moveit_msgs::msg::MotionPlanResponse MoveItPlanningPipeline::plan(
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
    plan_params.planner_id = node_ptr_->get_parameter("plan_request_params.planner_id").as_string();
    RCLCPP_INFO(LOGGER, "Planner ID: %s", plan_params.planner_id.c_str());
    plan_params.planning_pipeline = node_ptr_->get_parameter("plan_request_params.planning_pipeline").as_string();
    plan_params.planning_attempts = node_ptr_->get_parameter("plan_request_params.planning_attempts").as_int();
    plan_params.planning_time = node_ptr_->get_parameter("plan_request_params.planning_time").as_double();
    RCLCPP_INFO(LOGGER, "Planner time: %f", plan_params.planning_time);

    plan_params.max_velocity_scaling_factor = node_ptr_->get_parameter("plan_request_params.max_velocity_scaling_factor").as_double();
    plan_params.max_acceleration_scaling_factor = node_ptr_->get_parameter("plan_request_params.max_acceleration_scaling_factor").as_double();

    // Create planning component
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(group_name, moveit_cpp_);

    // create the moveit interface, and update the planning scene
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitor();
    auto planning_scene = planning_scene_monitor->getPlanningScene();

    planning_components->setStartStateToCurrentState();
    auto current_state = planning_scene->getCurrentStateNonConst();
    const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());

    // add position and orientation constraints
    moveit_msgs::msg::Constraints constraints;

    // Create position constraint for Cartesian bounds (loaded from YAML)
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = "link_0";
    position_constraint.link_name = "link_tool"; 
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.7, 0.85, 0.8};
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.5; // Center of the box in X direction
    box_pose.position.y = 0; // Center of the box in Y direction
    box_pose.position.z = 0.4; // Center of the box in Z direction
    // Box orientation: Identity quaternion (no rotation)
    box_pose.orientation.w = 1.0;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    position_constraint.constraint_region.primitives.emplace_back(box);
    position_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    position_constraint.weight = 0.7; // Weight of the constraint

    // Create orientation constraint
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = "link_0";
    orientation_constraint.link_name = "link_tool"; // Replace with the actual link name
    // Get the current orientation of the end effector
    const auto &current_orientation = link_state.rotation();
    Eigen::Quaterniond current_quat(current_orientation);
    RCLCPP_INFO(LOGGER, "Current orientation: %f, %f, %f, %f\n", current_quat.w(), current_quat.x(), current_quat.y(), current_quat.z());
    // Set the orientation constraint to the current orientation
    orientation_constraint.orientation.w = current_quat.w();
    orientation_constraint.orientation.x = current_quat.x();
    orientation_constraint.orientation.y = current_quat.y();
    orientation_constraint.orientation.z = current_quat.z();
    orientation_constraint.absolute_x_axis_tolerance = 0.5; // Tolerance values
    orientation_constraint.absolute_y_axis_tolerance = 0.5;
    orientation_constraint.absolute_z_axis_tolerance = 0.5;
    orientation_constraint.weight = 0.5; // Weight of the constraint

    // push the constraints into the constraints object
    constraints.orientation_constraints.push_back(orientation_constraint); // TOGGLE ORIENTATION CONSTRAINT
    constraints.position_constraints.emplace_back(position_constraint); // TOGGLE POSITION CONSTRAINT

    // Set the path constraints in the PlanningComponent
    planning_components->setPathConstraints(constraints); // TOGGLE CONSTRAINT

    // Check if the start state is valid as per the constraints
    bool is_valid = planning_scene->isStateValid(current_state, planning_components->getPlanningGroupName());
    RCLCPP_INFO(LOGGER, "State validity: %s", is_valid ? "valid" : "invalid");
    if (!is_valid)
    {
      // Print the Cartesian position of the current state
      const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());
      RCLCPP_ERROR(rclcpp::get_logger("planner"), "Start state is invalid!");
    }
    else
    {
      const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());
      RCLCPP_INFO(rclcpp::get_logger("planner"), "Start state is valid!");
    }

    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // Use system time (or RCL_ROS_TIME for simulation time)

    rclcpp::Time last_call_time = clock.now();

    // Set the goal to be only the position of the end effector
    auto final_goal = motion_plan_req.goal_constraints;
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "link_0";
    goal_pose.pose.position.x = final_goal[0].position_constraints[0].constraint_region.primitive_poses[0].position.x;
    goal_pose.pose.position.y = final_goal[0].position_constraints[0].constraint_region.primitive_poses[0].position.y;
    goal_pose.pose.position.z = final_goal[0].position_constraints[0].constraint_region.primitive_poses[0].position.z;
    goal_pose.pose.orientation.w = current_quat.w();
    goal_pose.pose.orientation.x = current_quat.x();
    goal_pose.pose.orientation.y = current_quat.y();
    goal_pose.pose.orientation.z = current_quat.z();

    bool ik_success;
    ik_success = goal_state_->setFromIK(joint_model_group_.get(), goal_pose.pose);
    RCLCPP_INFO(LOGGER, "IK success: %d", ik_success);
    std::vector<double> joint_values;
    goal_state_->copyJointGroupPositions(joint_model_group_.get(), joint_values);
    // for (size_t i = 0; i < joint_values.size(); ++i)
    // {
    //   RCLCPP_WARN(LOGGER, "Joint %ld: %f", i+1, joint_values[i]);
    // }

    // Convert joint values to RobotState
    moveit::core::RobotState goal_robot_state(robot_model_);
    goal_robot_state.setJointGroupPositions(joint_model_group_.get(), joint_values);
    goal_robot_state.update();

    planning_components->setGoal(goal_robot_state);   
    // planning_components->setGoal(goal_pose, "link_tool");

    // Plan first motion
    auto plan_solution = planning_components->plan(plan_params);
    int max_attempts = 5;
    int attempts = 0;
    while (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS && attempts < max_attempts)
    {
      RCLCPP_WARN(LOGGER, "Position planning attempt %d failed with error code: %d. Retrying...", attempts + 1, plan_solution.error_code.val);
      plan_solution = planning_components->plan(plan_params);
      attempts++;
    }

    // Get current time
    rclcpp::Time current_time = clock.now();
    
    // Calculate the time difference in seconds
    double time_difference = (current_time - last_call_time).seconds();

    RCLCPP_WARN(LOGGER, "Position planning time since last call: %.6f seconds", time_difference);
    last_call_time = clock.now();

    if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      response.error_code = plan_solution.error_code;
      RCLCPP_ERROR(LOGGER, "Position planning failed after %d attempts with error code: %d", attempts, plan_solution.error_code.val);
      return response;
    }

    // Set the planning start state to the last point in the trajectory of plan_solution
    moveit::core::RobotState last_state(robot_model_);
    last_state.setVariablePositions(plan_solution.trajectory->getLastWayPoint().getVariablePositions());
    planning_components->setStartState(last_state);
    
    // Plan the second part of the trajectory.
    // Add the target orientation to the goal
    goal_pose.pose.orientation = final_goal[0].orientation_constraints[0].orientation;
    
    ik_success = goal_state_->setFromIK(joint_model_group_.get(), goal_pose.pose);
    RCLCPP_INFO(LOGGER, "IK success: %d", ik_success);
    // std::vector<double> joint_values;
    goal_state_->copyJointGroupPositions(joint_model_group_.get(), joint_values);
    // for (size_t i = 0; i < joint_values.size(); ++i)
    // {
    //   RCLCPP_WARN(LOGGER, "Joint %d: %f", i+1, joint_values[i]);
    // }

    // Convert joint values to RobotState
    // moveit::core::RobotState goal_robot_state(robot_model_);
    goal_robot_state.setJointGroupPositions(joint_model_group_.get(), joint_values);
    goal_robot_state.update();

    planning_components->setGoal(goal_robot_state);   
    // planning_components->setGoal(goal_pose, "link_tool");

    //update the constraints to only include the position constraint
    moveit_msgs::msg::Constraints new_constraints;
    new_constraints.position_constraints.emplace_back(position_constraint);
    planning_components->setPathConstraints(new_constraints);

    // Plan the second part of the trajectory
    auto second_plan_solution = planning_components->plan(plan_params);
    while (second_plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS && attempts < max_attempts)
    {
      RCLCPP_WARN(LOGGER, "Orientation planning attempt %d failed with error code: %d. Retrying...", attempts + 1, plan_solution.error_code.val);
      second_plan_solution = planning_components->plan(plan_params);
      attempts++;
    }

    current_time = clock.now();

    // Calculate the time difference in seconds
    time_difference = (current_time - last_call_time).seconds();

    RCLCPP_WARN(LOGGER, "Orientation planning time since last call: %.6f seconds", time_difference);

    if (second_plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      response.error_code = second_plan_solution.error_code;
      RCLCPP_ERROR(LOGGER, "Orientation planning failed after %d attempts with error code: %d", attempts, plan_solution.error_code.val);
      return response;
    }

    // Create a MotionPlanResponse for puiblishing the trajectory
    response.trajectory_start = plan_solution.start_state;
    response.group_name = group_name;

    // Combine the first and second plan solutions
    plan_solution.trajectory->append(*second_plan_solution.trajectory, 0.0);

    // Transform the combined solution into MotionPlanResponse and publish it
    plan_solution.trajectory->getRobotTrajectoryMsg(response.trajectory);
    response.error_code = plan_solution.error_code;

    return response;
  }
  
} // namespace moveit::hybrid_planning


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::MoveItPlanningPipeline, moveit::hybrid_planning::GlobalPlannerInterface);
