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

#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}  // namespace

namespace moveit::hybrid_planning
{
  using namespace std::chrono_literals;

  HybridPlanningManager::HybridPlanningManager(const rclcpp::NodeOptions& options)
    : Node("hybrid_planning_manager", options), initialized_(false), stop_hybrid_planning_(true)
  {
    // stop_hybrid_planning_: default to true, so that the hybrid planning manager does not start planning until receive plan_flag from planning_state_sub_ 
    // Initialize hybrid planning component after construction
    RCLCPP_INFO(LOGGER, "auto declare %d", Node::get_node_options().automatically_declare_parameters_from_overrides());
    timer_ = this->create_wall_timer(1ms, [this]() {
      if (initialized_)
      {
        timer_->cancel();
      }
      else
      {
        if (!this->initialize())
        {
          const std::string error = "Failed to initialize global planner";
          timer_->cancel();
          throw std::runtime_error(error);
        }
        initialized_ = true;
      }
    });
  }

  bool HybridPlanningManager::initialize()
  {
    // Load planning logic plugin
    try
    {
      planner_logic_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerLogicInterface>>(
          "paradocs_planning", "moveit::hybrid_planning::PlannerLogicInterface");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception while creating planner logic plugin loader '%s'", ex.what());
    }
    std::string logic_plugin_name = "";
    if (this->has_parameter("planner_logic_plugin_name"))
    {
      this->get_parameter<std::string>("planner_logic_plugin_name", logic_plugin_name);
    }
    else
    {
      logic_plugin_name = this->declare_parameter<std::string>("planner_logic_plugin_name",
                                                              "moveit::hybrid_planning/MotionCompensation");
    }
    try
    {
      planner_logic_instance_ = planner_logic_plugin_loader_->createUniqueInstance(logic_plugin_name);
      if (!planner_logic_instance_->initialize(HybridPlanningManager::shared_from_this()))
      {
        throw std::runtime_error("Unable to initialize planner logic plugin");
      }
      RCLCPP_INFO(LOGGER, "Using planner logic interface '%s'", logic_plugin_name.c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception while loading planner logic '%s': '%s'", logic_plugin_name.c_str(), ex.what());
    }

    // Initialize local planning action client
    std::string local_planning_action_name = this->declare_parameter<std::string>("local_planning_action_name", "");
    this->get_parameter<std::string>("local_planning_action_name", local_planning_action_name);
    if (local_planning_action_name.empty())
    {
      RCLCPP_ERROR(LOGGER, "local_planning_action_name parameter was not defined");
      return false;
    }
    local_planner_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::LocalPlanner>(this, local_planning_action_name);
    if (!local_planner_action_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(LOGGER, "Local planner action server not available after waiting");
      return false;
    }

    // Initialize global planning action client
    std::string global_planning_action_name = this->declare_parameter<std::string>("global_planning_action_name", "");
    this->get_parameter<std::string>("global_planning_action_name", global_planning_action_name);
    if (global_planning_action_name.empty())
    {
      RCLCPP_ERROR(LOGGER, "global_planning_action_name parameter was not defined");
      return false;
    }
    global_planner_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::GlobalPlanner>(this, global_planning_action_name);
    if (!global_planner_action_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(LOGGER, "Global planner action server not available after waiting");
      return false;
    }

    // Initialize global solution subscriber (react is do nothing)
    // global_solution_sub_ = create_subscription<moveit_msgs::msg::MotionPlanResponse>(
    //     "global_trajectory", rclcpp::SystemDefaultsQoS(),
    //     [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& /* unused */) {
    //       // react is defined in a hybrid_planning_manager plugin
    //       ReactionResult reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE);
    //       if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    //       {
    //         RCLCPP_ERROR(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.error_message);
    //       }
    //     }
    // );

    // Initialize tracking goal subscriber
    tracking_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "tracked_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {

          // Threading is removed
          // if (long_callback_thread_.joinable())
          //   cancelHybridManagerGoals();
          // Execute the hybrid planner goal
          // long_callback_thread_ = std::thread(&HybridPlanningManager::executeHybridPlannerGoal, this, msg);
          if (!stop_hybrid_planning_)
          {
            executeHybridPlannerGoal(msg);
          }
        }
    );
    
    // Initialize planning state subscriber
    planning_state_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/lbr/plan_flag", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int32::ConstSharedPtr& msg) {
          RCLCPP_INFO(LOGGER, "Planning flag received: %d", msg->data);
          if (msg->data == 1)
            stop_hybrid_planning_ = false;
          else if (msg->data == 0) {
            stop_hybrid_planning_ = true;
            cancelHybridManagerGoals();
            // should reset previous goal, but set global_planner_started_ to false is the same effect
            // planner_logic_instance_->previous_goal_;
            planner_logic_instance_->reset();
          }
          else
            RCLCPP_ERROR(LOGGER, "Invalid planning state received: %d", msg->data);
        }
    );

    // For IK calculation
    this->declare_parameter<std::string>("robot_description_kinematics.arm.kinematics_solver", "pick_ik/PickIkPlugin");
    this->declare_parameter<double>("robot_description_kinematics.arm.kinematics_solver_timeout", 0.05);
    this->declare_parameter<int>("robot_description_kinematics.arm.kinematics_solver_attempts", 3);
    this->declare_parameter<std::string>("robot_description_kinematics.arm.mode", "global");
    this->declare_parameter<double>("robot_description_kinematics.arm.position_scale", 1.0); 
    this->declare_parameter<double>("robot_description_kinematics.arm.rotation_scale", 0.5);
    this->declare_parameter<double>("robot_description_kinematics.arm.position_threshold", 0.001);
    this->declare_parameter<double>("robot_description_kinematics.arm.orientation_threshold", 0.01);
    this->declare_parameter<double>("robot_description_kinematics.arm.cost_threshold", 0.001);
    this->declare_parameter<double>("robot_description_kinematics.arm.minimal_displacement_weight", 0.0);
    this->declare_parameter<double>("robot_description_kinematics.arm.gd_step_size", 0.0001);

    std::string robot_description = this->declare_parameter<std::string>("robot_description", "");
    this->get_parameter<std::string>("robot_description", robot_description);
    std::string planning_group = this->declare_parameter<std::string>("planning_group", "");
    this->get_parameter<std::string>("planning_group", planning_group);

    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
    // Get the robot model for IK
    robot_model_ = robot_model_loader.getModel();
    planning_group_ = planning_group;
    goal_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(
        goal_state_->getJointModelGroup(planning_group_));

    return true;
  }

  void HybridPlanningManager::stopGlobalPlanner()
  {

    auto global_future = global_planner_action_client_->async_cancel_all_goals();
    // if (global_future.valid()) {
    //   // Wait for the cancellation to finish
    //   // std::shared_future<T>::wait
    //   global_future.wait();
    // }
    RCLCPP_INFO(LOGGER, "stopGlobalPlanner success");

  }

  void HybridPlanningManager::stopLocalPlanner()
  {

    auto local_future = local_planner_action_client_->async_cancel_all_goals();
    // if (local_future.valid()) {
    //   // Wait for the cancellation to finish
    //   // std::shared_future<T>::wait
    //   local_future.wait();
    // }
    RCLCPP_INFO(LOGGER, "stopLocalPlanner success");

  }

  void HybridPlanningManager::cancelHybridManagerGoals() noexcept
  {
    stop_hybrid_planning_ = true;
    stopLocalPlanner();
    stopGlobalPlanner();
    RCLCPP_INFO(LOGGER, "cancelHybridManagerGoals finished");
  }

  void HybridPlanningManager::executeHybridPlannerGoal(
     const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& goal_handle) {

      // Reset the "stop" flag if it was set previously
      // stop_hybrid_planning_ = false;

      // Pass goal handle to class member
      hybrid_planning_goal_handle_ = std::move(goal_handle);

      // react is defined in a hybrid_planning_manager plugin
      ReactionResult reaction_result =
          planner_logic_instance_->react(HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED);
      if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
      }
    }

  bool HybridPlanningManager::sendGlobalPlannerAction()
  {

    auto global_goal_options = rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SendGoalOptions();

    // Add goal response callback
    global_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(LOGGER, "Global goal was rejected by server");
          } else {
            RCLCPP_INFO(LOGGER, "Global goal accepted by server");
          }
        };
    // Add result callback
    global_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::GlobalPlanner>::WrappedResult& global_result) {
          // Reaction result from the latest event, initialized with UNDEFINED, and will cause FAILURE
          ReactionResult reaction_result =
              ReactionResult(HybridPlanningEvent::UNDEFINED, "", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
          switch (global_result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL);
              break;
            case rclcpp_action::ResultCode::CANCELED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_CANCELED);
              break;
            case rclcpp_action::ResultCode::ABORTED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_PLANNING_ACTION_ABORTED);
              break;
            default:
              break;
          }
          // Abort hybrid planning if reaction fails
          if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
          }
        };

    // Fill in the global planning goal
    auto global_goal_msg = moveit_msgs::action::GlobalPlanner::Goal();
          
    // Create desired motion goal
    moveit_msgs::msg::MotionPlanRequest goal_motion_request;

    // goal_motion_request.goal_constraints.resize(1);
    // goal_motion_request.goal_constraints[0] =
    //     kinematic_constraints::constructGoalConstraints(*(goal_state_.get()), joint_model_group_.get());

    goal_motion_request.goal_constraints.resize(1);
    goal_motion_request.goal_constraints[0] =
      kinematic_constraints::constructGoalConstraints("link_tool", *(hybrid_planning_goal_handle_.get()));	

    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    // Single goal
    sequence_item.blend_radius = 0.0;
    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);

    global_goal_msg.motion_sequence = sequence_request;
    global_goal_msg.planning_group = planning_group_;

    if (stop_hybrid_planning_)
    {
      return false;
    }

    // Send global planning goal asynchronously
    auto goal_handle_future = global_planner_action_client_->async_send_goal(global_goal_msg, global_goal_options);
    return true;
  }

  bool HybridPlanningManager::sendLocalPlannerAction(bool isCompensation)
  {

    auto local_goal_options = rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SendGoalOptions();
    // rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr goal_handle;

    // Add goal response callback
    local_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(LOGGER, "Local goal was rejected by server");
          } else {
            RCLCPP_INFO(LOGGER, "Local goal accepted by server");
          }
        };

    // Add feedback callback
    local_goal_options.feedback_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& /*unused*/,
          const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Feedback>& local_planner_feedback) {
            // the feedback maybe COLLISION_AHEAD or LOCAL_PLANNER_STUCK
            // react is defined in a hybrid_planning_manager plugin
            ReactionResult reaction_result = planner_logic_instance_->react(local_planner_feedback->feedback);
            if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) 
            {
              RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
            }
          };

    // Add result callback to print the result
    local_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::WrappedResult& local_result) {
          // Reaction result from the latest event
          ReactionResult reaction_result =
              ReactionResult(HybridPlanningEvent::UNDEFINED, "", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
          switch (local_result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_SUCCESSFUL);
              break;
            case rclcpp_action::ResultCode::CANCELED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_CANCELED);
              break;
            case rclcpp_action::ResultCode::ABORTED:
              reaction_result = planner_logic_instance_->react(HybridPlanningEvent::LOCAL_PLANNING_ACTION_ABORTED);
              break;
            default:
              break;
          }
          // Abort hybrid planning if reaction fails
          if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
          }
        };

    if (stop_hybrid_planning_)
    {
      return false;
    }

    // Setup local goal
    auto local_goal_msg = moveit_msgs::action::LocalPlanner::Goal();

    if (isCompensation)
    {
      local_goal_msg.local_constraints.resize(1);
      local_goal_msg.local_constraints[0] = 
          kinematic_constraints::constructGoalConstraints(*(goal_state_.get()), 
                              joint_model_group_.get());
    } else {
      // send empty goal just to start hybrid_planning
      local_goal_msg.local_constraints.resize(0);
    }
    
    // Send local planning goal asynchronously
    auto goal_handle_future = local_planner_action_client_->async_send_goal(local_goal_msg, local_goal_options);
    return true;
  }

  bool HybridPlanningManager::calculateIK()
  {
    // Some alternative ways to give the goal info
    // 1. set goal_state using joint values
    // std::vector<double> joint_values = { 0.0, 0.0, 0.0, 1.57, 0.0, -1.57, 0.0};
    // goal_state_.setJointGroupPositions(joint_model_group_, joint_values);
    // 2. Skip goal state generates a constraint message intended to be used as a goal constraint for a given link. 
    // The full constraint will contain a PositionConstraint and a OrientationConstraint, 
    // constructed from the pose. A sphere will be used to represent the constraint region for the PositionConstraint.
    // goal_motion_request.goal_constraints.resize(1);
    // goal_motion_request.goal_constraints[0] =
    //     kinematic_constraints::constructGoalConstraints("link_tool", hybrid_planning_goal_handle_->pose);	

    // std::map<std::string, double> tmp_values;
    // std::vector<double> tmp_joint_values;

    // // Get all joint names in the JointModelGroup
    // const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();

    // // Populate the map with joint values from goal_state_
    // goal_state_->copyJointGroupPositions(joint_model_group_.get(), tmp_joint_values);  // Copy position to joint_position
    // int j = 0;
    // for (const std::string& joint_name : joint_names)
    // {
    //     tmp_values[joint_name] = tmp_joint_values[j];
    //     j++;
    // }

    // // Add the default state to JointModelGroup
    // joint_model_group_->addDefaultState("ik", tmp_values);

    bool success = goal_state_->setFromIK(joint_model_group_.get(), hybrid_planning_goal_handle_->pose);
    // TODO: check if this is necessary
    // goal_state_->update();
    RCLCPP_INFO(LOGGER, "IK success: %d", success);
    // std::vector<double> joint_values;
    // goal_state_->copyJointGroupPositions(joint_model_group_.get(), joint_values);
    // for (size_t i = 0; i < joint_values.size(); ++i)
    // {
    //   RCLCPP_INFO(LOGGER, "Joint %ld: %f", i+1, joint_values[i]);
    // }
    return success;
  }

}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)
