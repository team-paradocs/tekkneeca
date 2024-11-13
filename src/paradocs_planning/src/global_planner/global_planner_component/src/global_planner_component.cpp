/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

#include <moveit/global_planner/global_planner_component.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <chrono>
#include <thread>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
const auto JOIN_THREAD_TIMEOUT = std::chrono::seconds(1);
}  // namespace

namespace moveit::hybrid_planning
{
using namespace std::chrono_literals;

GlobalPlannerComponent::GlobalPlannerComponent(const rclcpp::NodeOptions& options)
  : node_{std::make_shared<rclcpp::Node>("global_planner_component", options)}
{
  RCLCPP_INFO(LOGGER, "Auto declare %d", node_->get_node_options().automatically_declare_parameters_from_overrides());
  RCLCPP_INFO(LOGGER, "Allow undeclare %d", node_->get_node_options().allow_undeclared_parameters());
  if (!initializeGlobalPlanner())
  {
    throw std::runtime_error("Failed to initialize Global Planner Component");
  }
}

bool GlobalPlannerComponent::initializeGlobalPlanner()
{

  declareParams(node_);

  // Initialize global planning request action server
  std::string global_planning_action_name = "";
  node_->get_parameter<std::string>("global_planning_action_name", global_planning_action_name);
  if (global_planning_action_name.empty())
  {
    RCLCPP_ERROR(LOGGER, "global_planning_action_name was not defined");
    return false;
  }
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  global_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::GlobalPlanner>(
      node_, global_planning_action_name,
      // Goal callback
      [this](const rclcpp_action::GoalUUID& /*unused*/,
             const std::shared_ptr<const moveit_msgs::action::GlobalPlanner::Goal>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received global planning goal request");
        // If another goal is active, cancel it and take the new goal
        if (long_callback_thread_.joinable())
        {
          // Try to join the execution thread
          // aync to try to call std::thread::join() and wait for it to finish
          // if after 1 sec still not able to join the thread, means the goal is still executing
          // so reject the new goal
          auto future = std::async(std::launch::async, &std::thread::join, &long_callback_thread_);
          if (future.wait_for(JOIN_THREAD_TIMEOUT) == std::future_status::timeout)
          {
            RCLCPP_WARN(LOGGER, "Another goal was running. don't publish the current goal and plan for the new global goal.");
            dont_pub_ = true;
          }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel global planning goal");
        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }
        dont_pub_ = true;
        // goal_handle will go to canceled state
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Execution callback
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        
        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }
        long_callback_thread_ = std::thread(&GlobalPlannerComponent::globalPlanningRequestCallback, this, goal_handle);
      },
      rcl_action_server_get_default_options(), cb_group_);

  global_trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::MotionPlanResponse>("global_trajectory", 1);

  // Load global planner plugin
  node_->get_parameter<std::string>("global_planner_name", planner_plugin_name_);
  try
  {
    global_planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<GlobalPlannerInterface>>(
        "paradocs_planning", "moveit::hybrid_planning::GlobalPlannerInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating global planner plugin loader: '%s'", ex.what());
    return false;
  }
  try
  {
    global_planner_instance_ = global_planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading global planner '%s': '%s'", planner_plugin_name_.c_str(), ex.what());
    return false;
  }

  // Initialize global planner plugin
  if (!global_planner_instance_->initialize(node_))
  {
    RCLCPP_ERROR(LOGGER, "Unable to initialize global planner plugin '%s'", planner_plugin_name_.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "Using global planner plugin '%s'", planner_plugin_name_.c_str());
  // default to publish the global plan
  dont_pub_ = false;
  return true;
}

void GlobalPlannerComponent::globalPlanningRequestCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& goal_handle)
{
  dont_pub_ = false;
  // Plan global trajectory
  moveit_msgs::msg::MotionPlanResponse planning_solution = global_planner_instance_->plan(goal_handle);

  // Send action response
  auto result = std::make_shared<moveit_msgs::action::GlobalPlanner::Result>();
  result->response = planning_solution;

  if (planning_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    if (dont_pub_)
    {
      // don't publish the global trajectory if dont_pub_ flag is true
      RCLCPP_WARN(LOGGER, "Global planning won't publish the global plan");
      return;
    }

    // Publish global planning solution to the local planner
    RCLCPP_INFO(LOGGER, "Global planning publish global plan");
    global_trajectory_pub_->publish(planning_solution);
    RCLCPP_INFO(LOGGER, "Global planning succeeded");
    goal_handle->succeed(result);
  }
  else
  {
    // planning failed
    goal_handle->abort(result);
  }

};
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::GlobalPlannerComponent)
