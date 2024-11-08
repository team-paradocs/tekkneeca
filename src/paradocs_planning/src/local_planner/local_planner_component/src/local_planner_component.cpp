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
                                                   config_.group_name))
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
    // pass in planning_scene_monitor_ for the plugin to check collision
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
        // always accept the new goal
        if (long_callback_thread_.joinable())
        {
          // Try to join the execution thread
          auto future = std::async(std::launch::async, &std::thread::join, &long_callback_thread_);
          if (future.wait_for(JOIN_THREAD_TIMEOUT) == std::future_status::timeout)
          {
            // should never happen because our thread ends very fast, won't reach timeout
            RCLCPP_WARN(LOGGER, "Another local goal was running. Rejecting the new hybrid planning goal.");
            // If another goal is active, cancel it and reject this goal
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Cancel callback
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel local planning execution");

        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }

        // auto local_trajectory_future = local_trajectory_action_client_->async_cancel_all_goals();
        // if (local_trajectory_future.valid()) {
        //   // Wait for the cancellation to finish
        //   local_trajectory_future.wait();
        // }
        reset();
        RCLCPP_INFO(LOGGER, "Local planning execution canceled");
        // state_ = LocalPlannerState::LOCAL_PLANNING_PAUSE;

        // aftet ACCEPT, the state will change EXECUTING -> CANCELING -> CANCELED
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Execution callback
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> goal_handle) {

        // assign local_planning_goal_handle_
        local_planning_goal_handle_ = std::move(goal_handle);

        size_t constraintSize = (local_planning_goal_handle_->get_goal())->local_constraints.size();
        RCLCPP_INFO(LOGGER, "Local planning goal size: %ld ", constraintSize);

        // If there is a constraint, then it is a compensation
        if (constraintSize > 0) {

          auto goal_constraint = (local_planning_goal_handle_->get_goal())->local_constraints[0];

          // RCLCPP_INFO(LOGGER, "Joint Constraint:");
          // RCLCPP_INFO(LOGGER, "Joint name %s,",
          //   goal_constraint.joint_constraints[0].joint_name.c_str());
          
          // RCLCPP_INFO(LOGGER, "position: %f, tolerance_above: %f, tolerance_below: %f, weight: %f",
          //   goal_constraint.joint_constraints[0].position,
          //   goal_constraint.joint_constraints[0].tolerance_above,
          //   goal_constraint.joint_constraints[0].tolerance_below,
          //   goal_constraint.joint_constraints[0].weight);           
            
          planning_scene_monitor_->updateSceneWithCurrentState();

          // Read current robot state
          const moveit::core::RobotState current_robot_state = [this] {
            planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
            return ls->getCurrentState();
          }();

          auto goal_state = std::make_shared<moveit::core::RobotState>(planning_scene_monitor_->getRobotModel());
          std::vector<double> joint_positions;
          for (size_t i = 0; i < goal_constraint.joint_constraints.size(); i++)
          {
            joint_positions.push_back(goal_constraint.joint_constraints[i].position);
          }
          
          goal_state->setVariablePositions(joint_positions.data());

          robot_trajectory::RobotTrajectory new_trajectory(planning_scene_monitor_->getRobotModel(), config_.group_name);

          RCLCPP_INFO(LOGGER, "state_ %d", static_cast<int>(state_.load()));
          if (state_ == LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY)
          {
            // cache the small motion
            temp_robot_trajectory_->addSuffixWayPoint(goal_state, 0.1);
          }
          else if (state_ == LocalPlannerState::LOCAL_PLANNING_ACTIVE)
          {
            new_trajectory.addSuffixWayPoint(goal_state, 0.1);
            // type 2: modify the current trajectory's last point to the new trajectory's last point
            trajectory_operator_instance_->setTrajectorySegment(new_trajectory, 2);
          }
          else if (state_ == LocalPlannerState::LOCAL_PLANNING_PAUSE)
          {
            // inactive
            // need to append start state and start local planer
            RCLCPP_INFO(LOGGER, "Add current_robot_state as part of compensation");
            new_trajectory.addSuffixWayPoint(std::make_shared<moveit::core::RobotState>(current_robot_state), 0.1);
            new_trajectory.addSuffixWayPoint(goal_state, 0.1);
            state_ = LocalPlannerState::LOCAL_PLANNING_ACTIVE;
            trajectory_operator_instance_->setTrajectorySegment(new_trajectory, 1);

            // Start a local planning loop
            // The thread and the timer can only be stated once
            if (long_callback_thread_.joinable())
            {
              long_callback_thread_.join();
            }

            auto local_planner_timer = [&]() {
              timer_ =
                  node_->create_wall_timer(1s / config_.local_planning_frequency, [this]() { return executeIteration(); });
            };
            long_callback_thread_ = std::thread(local_planner_timer);

          }
          else
          {
            RCLCPP_ERROR(LOGGER, "Unexpected state %d", static_cast<int>(state_.load()));
          }
        } 
        else
        {
          // Start a local planning loop
          // The thread and the timer can only be stated once
          if (long_callback_thread_.joinable())
          {
            long_callback_thread_.join();
          }

          auto local_planner_timer = [&]() {
            timer_ =
                node_->create_wall_timer(1s / config_.local_planning_frequency, [this]() { return executeIteration(); });
          };
          long_callback_thread_ = std::thread(local_planner_timer);
          state_ = LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
        }
      },
      rcl_action_server_get_default_options(), cb_group_);

  // Initialize global trajectory listener
  global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      config_.global_solution_topic, rclcpp::SystemDefaultsQoS(),
      [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& msg) {

        RCLCPP_INFO(LOGGER, "Received global trajectory");

        // Replace internal reference trajectory with received trajectory 
        robot_trajectory::RobotTrajectory new_trajectory(planning_scene_monitor_->getRobotModel(), msg->group_name);
        moveit::core::RobotState start_state(planning_scene_monitor_->getRobotModel());
        moveit::core::robotStateMsgToRobotState(msg->trajectory_start, start_state);
        new_trajectory.setRobotTrajectoryMsg(start_state, msg->trajectory);
        // replace current trajectory

        if (temp_robot_trajectory_->getWayPointCount() > 0)
        {
          // append the entire cached trajectory
          // new_trajectory.append(*(temp_robot_trajectory_.get()), 0.1);
          // append the last goal in cached trajectory

          // new_trajectory.removeWayPoint(new_trajectory.getWayPointCount() - 1);

          // Modify the last waypoint of the current trajectory
          if (new_trajectory.getWayPointCount() > 0)
          {

            // Create a temporary copy of the current trajectory
            auto tmp_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
                new_trajectory.getRobotModel(), new_trajectory.getGroupName());

            // Copy all but the last waypoint
            for (size_t i = 0; i < new_trajectory.getWayPointCount() - 1; ++i)
            {
                tmp_traj->addSuffixWayPoint(new_trajectory.getWayPoint(i), new_trajectory.getWayPointDurationFromPrevious(i));
            }

            // Add the last waypoint from the new trajectory
            tmp_traj->addSuffixWayPoint(temp_robot_trajectory_->getLastWayPoint(), 0.1);

            // Replace the reference trajectory with the modified one
            new_trajectory = *(tmp_traj.get());
          }
          else
          {
              throw std::runtime_error("Current trajectory is empty, cannot modify the last waypoint.");
          }

          new_trajectory.addSuffixWayPoint(temp_robot_trajectory_->getLastWayPoint(), 0.1);
          temp_robot_trajectory_->clear();
        }       

        // type 0: replace the current trajectory with the new trajectory
        trajectory_operator_instance_->setTrajectorySegment(new_trajectory, 0);

        // TODO: decide whether we display the global trajectory
        // moveit_msgs::msg::DisplayTrajectory display_trajectory;
        // moveit::core::robotStateToRobotStateMsg(start_state, display_trajectory.trajectory_start);
        // moveit_msgs::msg::RobotTrajectory msg_trajectory;
        // new_trajectory.getRobotTrajectoryMsg(msg_trajectory); 
        // display_trajectory.trajectory.push_back(msg_trajectory);
        // // display_publisher->publish(display_trajectory);
        // visual_tools_->publishTrajectoryLine(new_trajectory, joint_model_group_.get());
        // visual_tools_->trigger();

        // Feedback is only send when the hybrid planning architecture should react to a discrete event that occurred
        // We don't want the hybrid planning manager to react to this so we don't send feedback
        // when the reference trajectory is updated
        // if (!local_planner_feedback_->feedback.empty())
        // {
        //   local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        // }

        // Update local planner state
        if (long_callback_thread_.joinable())
        {
          long_callback_thread_.join();
        }
        
        state_ = LocalPlannerState::LOCAL_PLANNING_ACTIVE;

        auto local_planner_timer = [&]() {
          timer_ =
              node_->create_wall_timer(1s / config_.local_planning_frequency, [this]() { return executeIteration(); });
        };
        long_callback_thread_ = std::thread(local_planner_timer);
      }
  );

  // Initialize local solution publisher
  RCLCPP_INFO(LOGGER, "Using '%s' as local solution topic type", config_.local_solution_topic_type.c_str());

  // Local solution publisher is defined by the local constraint solver plugin
  local_trajectory_action_client_ = 
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, config_.local_solution_topic);

  // Wait until the action server is available
  while (!local_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 5000 /* ms */, 
      "Waiting for FollowJointTrajectory action server to be available...");
  }
  RCLCPP_INFO(LOGGER, "FollowJointTrajectory Action server available.");

  // TODO: decide whether we display the planned trajectory
  // display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/lbr/display_planned_path", 1);
  // RCLCPP_INFO(LOGGER, "Will display the planned trajectory");
  // joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(planning_scene_monitor_->getRobotModel()->getJointModelGroup(config_.group_name));
  // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "link_0", "local_planner_traj", planning_scene_monitor_);
  // visual_tools_->deleteAllMarkers();
  // visual_tools_->loadRemoteControl();

  // this is a dummy goal to publish feedback to manager's client

  // initialize the temp_robot_trajectory_
  temp_robot_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene_monitor_->getRobotModel(), config_.group_name);
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
    case LocalPlannerState::LOCAL_PLANNING_PAUSE:
      // TODO: decide if we need to remove the reference_trajectory here
      // Do nothing for now
      // Keep monitor the motion
      // reset();
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
        RCLCPP_INFO(LOGGER, "Local planner reached the goal");
        local_planning_goal_handle_->succeed(result);
        reset();
        state_ = LocalPlannerState::LOCAL_PLANNING_PAUSE;
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
          local_trajectory, local_solution);

      // Feedback is only send when the hybrid planning architecture should react to a discrete event
      if (!local_planner_feedback_->feedback.empty())
      {
        // stucked or collision ahead
        local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        // TODO: decide if we need to change state here
        return;
      }

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
              // RCLCPP_INFO(LOGGER, "Goal accepted by the joint trajectory controller action server, executing...");
          }
        };        

      goal_options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr /*unused*/,
                                              const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> /*unused*/) {
          // RCLCPP_INFO(LOGGER, "Received feedback: %f", feedback->desired.positions[0]);  // Access feedback values
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

      return;
    }
    default:
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->error_message = "Unexpected failure.";
      local_planning_goal_handle_->abort(result);
      RCLCPP_ERROR(LOGGER, "Local planner somehow failed");
      reset();
      return;
    }
  }
}

void LocalPlannerComponent::reset()
{
  local_constraint_solver_instance_->reset();
  trajectory_operator_instance_->reset();
  temp_robot_trajectory_->clear();
  timer_->cancel();
  state_ = LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
}
}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::LocalPlannerComponent)
