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

#include <moveit/local_planner/local_planner_component.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/constraints.hpp>

namespace moveit::hybrid_planning
{
using namespace std::chrono_literals;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
const auto JOIN_THREAD_TIMEOUT = std::chrono::seconds(1);

// If the trajectory progress reaches more than 0.X the global goal state is considered as reached
constexpr float PROGRESS_THRESHOLD = 0.995;
}  // namespace

LocalPlannerComponent::LocalPlannerComponent(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("local_planner_component", options) }
{
  state_ = LocalPlannerState::UNCONFIGURED;
  local_planner_feedback_ = std::make_shared<moveit_msgs::action::LocalPlanner::Feedback>();

  if (!this->initialize())
  {
    throw std::runtime_error("Failed to initialize local planner component");
  }
}

bool LocalPlannerComponent::initialize()
{
  // Load planner parameter
  config_.load(node_);

  // Validate config
  // if (config_.local_solution_topic_type == "std_msgs/Float64MultiArray")
  // {
  //   if ((config_.publish_joint_positions && config_.publish_joint_velocities) ||
  //       (!config_.publish_joint_positions && !config_.publish_joint_velocities))
  //   {
  //     RCLCPP_ERROR(LOGGER, "When publishing a std_msgs/Float64MultiArray, you must select positions OR velocities. "
  //                          "Enabling both or none is not possible!");
  //     return false;
  //   }
  // }

  // Configure planning scene monitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer_, "local_planner/planning_scene_monitor");
  if (!planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_ERROR(LOGGER, "Unable to configure planning scene monitor");
    return false;
  }

  // Start state and scene monitors
  planning_scene_monitor_->startSceneMonitor(config_.monitored_planning_scene_topic);
  planning_scene_monitor_->startWorldGeometryMonitor(config_.collision_object_topic);
  planning_scene_monitor_->startStateMonitor(config_.joint_states_topic);

  // Load trajectory operator plugin
  try
  {
    trajectory_operator_loader_ = std::make_unique<pluginlib::ClassLoader<TrajectoryOperatorInterface>>(
        "paradocs_planning", "moveit::hybrid_planning::TrajectoryOperatorInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating trajectory operator plugin loader: '%s'", ex.what());
    return false;
  }
  try
  {
    trajectory_operator_instance_ =
        trajectory_operator_loader_->createUniqueInstance(config_.trajectory_operator_plugin_name);
    if (!trajectory_operator_instance_->initialize(node_, planning_scene_monitor_->getRobotModel(),
                                                   config_.group_name))  // TODO(sjahr) add default group param
      throw std::runtime_error("Unable to initialize trajectory operator plugin");
    RCLCPP_INFO(LOGGER, "Using trajectory operator interface '%s'", config_.trajectory_operator_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading trajectory operator '%s': '%s'",
                 config_.trajectory_operator_plugin_name.c_str(), ex.what());
    return false;
  }

  // Load local constraint solver
  try
  {
    local_constraint_solver_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<LocalConstraintSolverInterface>>(
        "paradocs_planning", "moveit::hybrid_planning::LocalConstraintSolverInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while creating constraint solver plugin loader '%s'", ex.what());
    return false;
  }
  try
  {
    local_constraint_solver_instance_ =
        local_constraint_solver_plugin_loader_->createUniqueInstance(config_.local_constraint_solver_plugin_name);
    if (!local_constraint_solver_instance_->initialize(node_, planning_scene_monitor_, config_.group_name))
      throw std::runtime_error("Unable to initialize constraint solver plugin");
    RCLCPP_INFO(LOGGER, "Using constraint solver interface '%s'", config_.local_constraint_solver_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading constraint solver '%s': '%s'",
                 config_.local_constraint_solver_plugin_name.c_str(), ex.what());
    return false;
  }

  // Initialize local planning request action server
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  local_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::LocalPlanner>(
      node_, config_.local_planning_action_name,
      // Goal callback
      [this](const rclcpp_action::GoalUUID& /*unused*/,
             const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received local planning goal request");
        // If another goal is active, cancel it and reject this goal
        if (long_callback_thread_.joinable())
        {
          // Try to terminate the execution thread
          auto future = std::async(std::launch::async, &std::thread::join, &long_callback_thread_);
          if (future.wait_for(JOIN_THREAD_TIMEOUT) == std::future_status::timeout)
          {
            RCLCPP_WARN(LOGGER, "Another goal was running. Rejecting the new hybrid planning goal.");
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel local planning goal");
        state_ = LocalPlannerState::LOCAL_PLANNING_CANCELED;
        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }

        auto local_trajectory_future = local_trajectory_action_client_->async_cancel_all_goals();
        if (local_trajectory_future.valid()) {
          // Wait for the cancellation to finish
          local_trajectory_future.wait();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Execution callback
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> goal_handle) {
        local_planning_goal_handle_ = std::move(goal_handle);
        // Check if a previous goal was running and needs to be cancelled.
        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }
        // Start a local planning loop.
        // This needs to return quickly to avoid blocking the executor, so run the local planner in a new thread.
        auto local_planner_timer = [&]() {
          timer_ =
              node_->create_wall_timer(1s / config_.local_planning_frequency, [this]() { return executeIteration(); });
        };
        long_callback_thread_ = std::thread(local_planner_timer);
      },
      rcl_action_server_get_default_options(), cb_group_);

  // Initialize global trajectory listener
  global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      config_.global_solution_topic, rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& msg) {
        // Add received trajectory to internal reference trajectory
        robot_trajectory::RobotTrajectory new_trajectory(planning_scene_monitor_->getRobotModel(), msg->group_name);
        moveit::core::RobotState start_state(planning_scene_monitor_->getRobotModel());
        moveit::core::robotStateMsgToRobotState(msg->trajectory_start, start_state);
        new_trajectory.setRobotTrajectoryMsg(start_state, msg->trajectory);
        *local_planner_feedback_ = trajectory_operator_instance_->addTrajectorySegment(new_trajectory);

        // Feedback is only send when the hybrid planning architecture should react to a discrete event that occurred
        // when the reference trajectory is updated
        if (!local_planner_feedback_->feedback.empty())
        {
          local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        }

        // Update local planner state
        state_ = LocalPlannerState::LOCAL_PLANNING_ACTIVE;
      });

  // Initialize local solution publisher
  RCLCPP_INFO(LOGGER, "Using '%s' as local solution topic type", config_.local_solution_topic_type.c_str());
  // if (config_.local_solution_topic_type == "trajectory_msgs/msg/JointTrajectory")
  // {
  //   local_trajectory_publisher_ =
  //       node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(config_.local_solution_topic, 1);
  // }
  // else if (config_.local_solution_topic_type == "std_msgs/Float64MultiArray")
  // {
  //   local_solution_publisher_ =
  //       node_->create_publisher<std_msgs::msg::Float64MultiArray>(config_.local_solution_topic, 1);
  // }
  // else if (config_.local_solution_topic_type == "control_msgs/action/FollowJointTrajectory")
  // {

  // Local solution publisher is defined by the local constraint solver plugin
  local_trajectory_action_client_ = 
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, config_.local_solution_topic);

  // Wait until the action server is available
  while (!local_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 5000 /* ms */, 
      "Waiting for FollowJointTrajectory action server to be available...");
  }
  RCLCPP_INFO(LOGGER, "FollowJointTrajectory Action server available.");

  // }

  state_ = LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
  return true;
}

void LocalPlannerComponent::executeIteration()
{
  auto result = std::make_shared<moveit_msgs::action::LocalPlanner::Result>();

  // Do different things depending on the planner's internal state
  switch (state_)
  {
    // Wait for global solution to be published
    case LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY:
      // Do nothing
      return;
    case LocalPlannerState::LOCAL_PLANNING_CANCELED:
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      result->error_message = "Local planner is in an canceled state. Resetting.";
      local_planning_goal_handle_->canceled(result);
      reset();
      return;
    // Notify action client that local planning failed
    case LocalPlannerState::ABORT:
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->error_message = "Local planner is in an aborted state. Resetting.";
      local_planning_goal_handle_->abort(result);
      reset();
      return;
    }
    // If the planner received an action request and a global solution it starts to plan locally
    case LocalPlannerState::LOCAL_PLANNING_ACTIVE:
    {
      planning_scene_monitor_->updateSceneWithCurrentState();

      // Read current robot state
      const moveit::core::RobotState current_robot_state = [this] {
        planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
        return ls->getCurrentState();
      }();

      // Check if the global goal is reached
      if (trajectory_operator_instance_->getTrajectoryProgress(current_robot_state) > PROGRESS_THRESHOLD)
      {
        local_planning_goal_handle_->succeed(result);
        reset();
        return;
      }

      // Get local goal trajectory to follow
      robot_trajectory::RobotTrajectory local_trajectory =
          robot_trajectory::RobotTrajectory(planning_scene_monitor_->getRobotModel(), config_.group_name);
      *local_planner_feedback_ =
          trajectory_operator_instance_->getLocalTrajectory(current_robot_state, local_trajectory);

      // Feedback is only sent when the hybrid planning architecture should react to a discrete event that occurred
      // during the identification of the local planning problem
      if (!local_planner_feedback_->feedback.empty())
      {
        local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        RCLCPP_ERROR(LOGGER, "Local planner somehow failed");
        reset();
        return;
      }

      // Solve local planning problem
      trajectory_msgs::msg::JointTrajectory local_solution;

      // Feedback is only send when the hybrid planning architecture should react to a discrete event that occurred
      // while computing a local solution
      *local_planner_feedback_ = local_constraint_solver_instance_->solve(
          local_trajectory, local_planning_goal_handle_->get_goal(), local_solution);

      // Feedback is only send when the hybrid planning architecture should react to a discrete event
      if (!local_planner_feedback_->feedback.empty())
      {
        local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        return;
      }

      // Use a configurable message interface like MoveIt servo
      // (See https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/src/servo_calcs.cpp)
      // Format outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or joint positions/velocities in form of std_msgs/Float64MultiArray).

      // if (config_.local_solution_topic_type == "trajectory_msgs/msg/JointTrajectory")
      // {
      //   local_trajectory_publisher_->publish(local_solution);
      //   // RCLCPP_INFO(LOGGER, "JointTrajectory Published");

      // }
      // else if (config_.local_solution_topic_type == "std_msgs/Float64MultiArray")
      // {
      //   // Transform "trajectory_msgs/JointTrajectory" to "std_msgs/Float64MultiArray"
      //   auto joints = std::make_unique<std_msgs::msg::Float64MultiArray>();
      //   if (!local_solution.points.empty())
      //   {
      //     if (config_.publish_joint_positions)
      //     {
      //       joints->data = local_solution.points[0].positions;
      //     }
      //     else if (config_.publish_joint_velocities)
      //     {
      //       joints->data = local_solution.points[0].velocities;
      //     }
      //   }
      //   local_solution_publisher_->publish(std::move(joints));
      //   // RCLCPP_INFO(LOGGER, "Float64MultiArray Published");
      // }
      // else if (config_.local_solution_topic_type == "control_msgs/action/FollowJointTrajectory")
      // {
        
      // Local solution publisher is defined by the local constraint solver plugin
      auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

      // Assign trajectory to goal message
      goal_msg.trajectory = local_solution;

      // Send goal to action server
      auto goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
      
      goal_options.goal_response_callback = 
        [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle) {
          // auto goal_handle = goalHandle.get();
          if (!goal_handle) {
              RCLCPP_ERROR(LOGGER, "Goal was rejected by the joint trajectory controller action server.");
          } else {
              RCLCPP_INFO(LOGGER, "Goal accepted by the joint trajectory controller action server, executing...");
          }
        };        

      goal_options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr /*unused*/,
                                              const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
          RCLCPP_INFO(LOGGER, "Received feedback: %f", feedback->desired.positions[0]);  // Access feedback values
      };

      goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result) {
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
      local_trajectory_action_client_->async_send_goal(goal_msg, goal_options);

      // }
      return;
    }
    default:
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->error_message = "Unexpected failure.";
      local_planning_goal_handle_->abort(result);
      // RCLCPP_ERROR(LOGGER, "Local planner somehow failed");  // TODO(sjahr) Add more detailed failure information
      reset();
      return;
    }
  }
}

void LocalPlannerComponent::reset()
{
  local_constraint_solver_instance_->reset();
  trajectory_operator_instance_->reset();
  timer_->cancel();
  state_ = LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
}
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::LocalPlannerComponent)
