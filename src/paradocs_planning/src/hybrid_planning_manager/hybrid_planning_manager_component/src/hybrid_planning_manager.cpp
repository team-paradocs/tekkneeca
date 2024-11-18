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
    : Node("hybrid_planning_manager", options), initialized_(false), planner_state_(0)
  {
    // planner_state_: default to 0, so that the hybrid planning manager does not start planning until receive plan_flag from planning_state_sub_ 
    // Initialize hybrid planning component after construction
    RCLCPP_INFO(LOGGER, "Auto declare %d", Node::get_node_options().automatically_declare_parameters_from_overrides());
    RCLCPP_INFO(LOGGER, "Allow undeclare %d", Node::get_node_options().allow_undeclared_parameters());
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

    declareParams();

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

    std::string logic_plugin_name;
    this->get_parameter<std::string>("planner_logic_plugin_name", logic_plugin_name);
    if (logic_plugin_name.empty())
    {
      RCLCPP_ERROR(LOGGER, "logic_plugin_name parameter was not defined");
      return false;
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
    std::string local_planning_action_name;
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
    std::string global_planning_action_name;
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

    // Initialize global solution subscriber (react is to cache the global solution)
    global_solution_sub_ = create_subscription<moveit_msgs::msg::MotionPlanResponse>(
        "global_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& msg) {
          moveit::core::RobotState start_state(robot_model_);
          moveit::core::robotStateMsgToRobotState(msg->trajectory_start, start_state);
          cache_global_trajectory_->setRobotTrajectoryMsg(start_state, msg->trajectory);
          RCLCPP_INFO(LOGGER, "Global solution cached");
          // react is defined in a hybrid_planning_manager plugin
          ReactionResult reaction_result = planner_logic_instance_->react(HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE);
          if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.error_message);
          }
        }
    );

    // stop_execution_publisher_ = this->create_publisher<std_msgs::msg::Bool>("stop_execution", rclcpp::SystemDefaultsQoS());

    start_drilling_publisher_ = this->create_publisher<std_msgs::msg::String>("drill_commands", 10);

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
            // planning, the execution or not is taken care by stop_execution_publisher
            executeHybridPlannerGoal();
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
            executeHybridPlannerGoal();
          }
          else if (planner_state_ == 0) {
            // stop planning stop execution
            drill_state_ = 0;
            cancelHybridManagerGoals();
            planner_logic_instance_->reset();
          }
          else if (planner_state_ == 2) {

            // start drill motion
            if (drill_state_ == 0) {
              drill_state_ = 1;
              ReactionResult reaction_result = planner_logic_instance_->react(HybridPlanningEvent::DRILLING_REQUEST_RECEIVED);
              if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) 
              {
                RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
              }
            }
          }
          else if (msg->data == 3) 
          {
            // hard reset and go back to home
            cancelHybridManagerGoals();
            planner_logic_instance_->reset();
            drill_state_ = 6;
            drillMotion();

          }
          else
            RCLCPP_ERROR(LOGGER, "Invalid planning flag received: %d", msg->data);
        }
    );

    std::string robot_description_name;
    this->get_parameter<std::string>("robot_description_name", robot_description_name);
    std::string planning_group;
    this->get_parameter<std::string>("planning_group", planning_group);

    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), robot_description_name);
    // Get the robot model for IK
    robot_model_ = robot_model_loader.getModel();
    planning_group_ = planning_group;
    goal_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(
        goal_state_->getJointModelGroup(planning_group_));
    cache_global_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_);
    drillWayPointConstraints = std::vector<moveit_msgs::msg::Constraints>();
    drill_pose_goal_handle_ = nullptr;
    hybrid_planning_goal_handle_ = nullptr;
    drill_state_ = 0;
    return true;
  }

  void HybridPlanningManager::stopGlobalPlanner()
  {

    auto global_future = global_planner_action_client_->async_cancel_all_goals();
    RCLCPP_INFO(LOGGER, "stopGlobalPlanner success");

  }

  void HybridPlanningManager::stopLocalPlanner()
  {

    auto local_future = local_planner_action_client_->async_cancel_all_goals();
    RCLCPP_INFO(LOGGER, "stopLocalPlanner success");

  }

  void HybridPlanningManager::cancelHybridManagerGoals() noexcept
  {
    stopLocalPlanner();
    stopGlobalPlanner();
    RCLCPP_INFO(LOGGER, "cancelHybridManagerGoals finished");
  }

  void HybridPlanningManager::executeHybridPlannerGoal() 
  {
    // react is defined in a hybrid_planning_manager plugin
    ReactionResult reaction_result =
        planner_logic_instance_->react(HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED);
    if (reaction_result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) 
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Hybrid Planning Manager failed to react to " << reaction_result.event << " with error: " << reaction_result.error_message);
    }
  }

  bool HybridPlanningManager::sendGlobalPlannerAction(bool is_drill)
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

    if (is_drill) {
      goal_motion_request.goal_constraints.resize(drillWayPointConstraints.size());
      goal_motion_request.goal_constraints = drillWayPointConstraints;

    } else {
      goal_motion_request.goal_constraints.resize(1);
      goal_motion_request.goal_constraints[0] =
          kinematic_constraints::constructGoalConstraints(*(goal_state_.get()), joint_model_group_.get());
    }

    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    // Single goal
    sequence_item.blend_radius = 0.0;
    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);
    global_goal_msg.motion_sequence = sequence_request;
    global_goal_msg.planning_group = planning_group_;
    if (planner_state_ == 0)
    {
      return false;
    }
    // Send global planning goal asynchronously
    auto goal_handle_future = global_planner_action_client_->async_send_goal(global_goal_msg, global_goal_options);
    return true;
  }

  bool HybridPlanningManager::sendLocalPlannerAction(int type)
  {
    // RCLCPP_INFO(LOGGER, "In sendLocalPlannerAction function, type: %d", type);
    // 0: do nothing for now, can be another signal in the future
    // 1: global traj, don't start exe loop just chage state
    // 2: compensate traj, don't start exe loop just chage state

    auto local_goal_options = rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SendGoalOptions();
    // rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr goal_handle;

    // Add goal response callback
    local_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::LocalPlanner>::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(LOGGER, "Local goal was rejected by server");
          } else {
            // RCLCPP_INFO(LOGGER, "Local goal accepted by server");
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

    if (planner_state_ == 0)
    {
      return false;
    }

    // Setup local goal
    auto local_goal_msg = moveit_msgs::action::LocalPlanner::Goal();

    if (type == 2)
    {
      local_goal_msg.local_constraints.resize(1);
      local_goal_msg.local_constraints[0] = 
          kinematic_constraints::constructGoalConstraints(*(goal_state_.get()), 
                              joint_model_group_.get());
    } 
    else if (type == 1)
    {
      // send cache_global_trajectory_ to local planner
      size_t trajSize = cache_global_trajectory_->getWayPointCount();
      // RCLCPP_INFO(LOGGER, "trajSize, %ld", trajSize);

      // Loop through each waypoint in the trajectory
      for (size_t i = 0; i < trajSize; ++i)
      {
        moveit_msgs::msg::Constraints target_constraints;

        // Get the current robot state for the waypoint
        const moveit::core::RobotState& waypoint_state = cache_global_trajectory_->getWayPoint(i);

        // Create a joint constraint for this joint
        moveit_msgs::msg::JointConstraint joint_constraint;

        const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
        for (const std::string& joint_name : joint_names)
        {
          // Get the position of the joint in the current waypoint
          double joint_position = waypoint_state.getVariablePosition(joint_name);
          joint_constraint.joint_name = joint_name;
          joint_constraint.position = joint_position;
          joint_constraint.tolerance_above = 0.01;
          joint_constraint.tolerance_below = 0.01;
          joint_constraint.weight = 1.0;
          target_constraints.joint_constraints.emplace_back(joint_constraint);
        }

        local_goal_msg.local_constraints.emplace_back(target_constraints);
      }

      // sanity check
      // RCLCPP_INFO(LOGGER, "local_goal_msg.local_constraints.size() %ld ", local_goal_msg.local_constraints.size());
      // RCLCPP_INFO(LOGGER, "local_goal_msg.local_constraints[0].joint_constraints.size() %ld ", local_goal_msg.local_constraints[0].joint_constraints.size());
    } 
    else
    {
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

    if (hybrid_planning_goal_handle_ == nullptr)
    {
      RCLCPP_ERROR(LOGGER, "hybrid_planning_goal_handle_ is nullptr");
      return false;
    }

    bool success = goal_state_->setFromIK(joint_model_group_.get(), hybrid_planning_goal_handle_->pose);
    if (!success)
    {
      RCLCPP_ERROR(LOGGER, "IK failed");
    }
    return success;
  }

  geometry_msgs::msg::PoseStamped HybridPlanningManager::computeOffsetPose(const geometry_msgs::msg::PoseStamped::SharedPtr& targetPose, float offset = 0.02)
  {
    float x_hard_coded_offset = 0.00;
    float y_hard_coded_offset = 0.00;

    // Calculate the final drill position
    // Convert the quaternion to a rotation matrix
    Eigen::Quaterniond quat(targetPose->pose.orientation.w,
                            targetPose->pose.orientation.x,
                            targetPose->pose.orientation.y,
                            targetPose->pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Create a vector representing the direction to move in
    Eigen::Vector3d direction(0, 0, offset);

    // Multiply the rotation matrix by the direction vector
    Eigen::Vector3d result = rotation_matrix * direction;

    // Add the result to the original position to get the new position
    geometry_msgs::msg::PoseStamped newPose;
    newPose.pose.position.x = targetPose->pose.position.x + result(0) + x_hard_coded_offset;
    newPose.pose.position.y = targetPose->pose.position.y + result(1) + y_hard_coded_offset;
    newPose.pose.position.z = targetPose->pose.position.z + result(2);

    // The orientation remains the same
    newPose.pose.orientation = targetPose->pose.orientation;

    newPose.header.frame_id = targetPose->header.frame_id;
    newPose.header.stamp = targetPose->header.stamp;

    return newPose;
  }

  void HybridPlanningManager::drillMotion()
  {
    int stage = drill_state_.load();

    RCLCPP_INFO(LOGGER, "In function drillMotion, drill_state_: %d", stage);

    // legacy from static_obstacles
    // cartesian planning parameters
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    if (drill_pose_goal_handle_ == nullptr)
    {
      RCLCPP_ERROR(LOGGER, "drill_pose_goal_handle_ is nullptr");
      drill_state_ = 0;
      return;
    }

    geometry_msgs::msg::PoseStamped preDrillPose = computeOffsetPose(drill_pose_goal_handle_, -0.05);
    moveit_msgs::msg::Constraints predrill_constraints =
      kinematic_constraints::constructGoalConstraints("link_tool", preDrillPose);

    geometry_msgs::msg::PoseStamped startDrillPose = computeOffsetPose(drill_pose_goal_handle_, -0.015);
    moveit_msgs::msg::Constraints start_drill_constraints =
      kinematic_constraints::constructGoalConstraints("link_tool", startDrillPose);

    geometry_msgs::msg::PoseStamped touchPose = computeOffsetPose(drill_pose_goal_handle_, -0.001);
    moveit_msgs::msg::Constraints touch_constraints =
      kinematic_constraints::constructGoalConstraints("link_tool", touchPose);

    geometry_msgs::msg::PoseStamped endPose = computeOffsetPose(drill_pose_goal_handle_, 0.030);

    geometry_msgs::msg::PoseStamped offDrillPose = computeOffsetPose(drill_pose_goal_handle_, -0.015);
    moveit_msgs::msg::Constraints off_drill_constraints =
      kinematic_constraints::constructGoalConstraints("link_tool", offDrillPose);

    std::vector<double> joint_values = { 0.0, 0.0, 0.0, 1.57, 0.0, -1.57, 0.0};
    moveit::core::RobotState temp_state(robot_model_);
    temp_state.setJointGroupPositions(joint_model_group_.get(), joint_values);
    moveit_msgs::msg::Constraints home_constraints =
      kinematic_constraints::constructGoalConstraints(temp_state, joint_model_group_.get());

    drillWayPointConstraints.clear();

    if (stage == 1)
    {
      // go to predrill
      drillWayPointConstraints.push_back(predrill_constraints);
    }
    else if (stage == 2)
    {
      drillWayPointConstraints.push_back(start_drill_constraints);
    }
    // between stage 2 and 3, we switch on the drill
    else if (stage == 3)
    {
      // drill in

      // drillWayPointConstraints.push_back(touch_constraints);

      int drillPoints = 25;
      // Interpolate drill_points number of points between start and end pose
      // std::stack<moveit_msgs::msg::Constraints> interpolatedConstraintsStack;

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

        // interpolatedConstraintsStack.push(interpolatedConstraints);
        drillWayPointConstraints.push_back(interpolatedConstraints);
      }

      RCLCPP_WARN(LOGGER, "drillWayPointConstraints.size() %ld", drillWayPointConstraints.size());

      // drill out 
      // while (!interpolatedConstraintsStack.empty()) {
      //   drillWayPointConstraints.push_back(interpolatedConstraintsStack.top());
      //   interpolatedConstraintsStack.pop();
      // }

      // go back to touch
      // drillWayPointConstraints.push_back(touch_constraints);
    }
    else if (stage == 4)
    {
      int drillPoints = 25;
      
     //drill out
      for (int i = 0; i <= drillPoints; i++) {
        geometry_msgs::msg::PoseStamped interpolatedPose;
        float ratio = static_cast<float>(i) / drillPoints;
        // Interpolate position
        interpolatedPose.pose.position.x = endPose.pose.position.x + ratio * (touchPose.pose.position.x - endPose.pose.position.x);
        interpolatedPose.pose.position.y = endPose.pose.position.y + ratio * (touchPose.pose.position.y - endPose.pose.position.y);
        interpolatedPose.pose.position.z = endPose.pose.position.z + ratio * (touchPose.pose.position.z - endPose.pose.position.z);
        // Orientation remains the same
        interpolatedPose.pose.orientation = endPose.pose.orientation;
        // Copy header
        interpolatedPose.header.frame_id = endPose.header.frame_id;
        interpolatedPose.header.stamp = endPose.header.stamp;

        moveit_msgs::msg::Constraints interpolatedConstraints =
          kinematic_constraints::constructGoalConstraints("link_tool", interpolatedPose);

        // interpolatedConstraintsStack.push(interpolatedConstraints);
        drillWayPointConstraints.push_back(interpolatedConstraints);
      }

      RCLCPP_WARN(LOGGER, "drillWayPointConstraints.size() %ld", drillWayPointConstraints.size());
    }
    else if (stage == 5)
    {
      // go to off drillpose
      drillWayPointConstraints.push_back(off_drill_constraints);
    }
    // between stage 4 and 5, we switch off the drill
    else if (stage == 6)
    {
      // go back to predrill
      drillWayPointConstraints.push_back(predrill_constraints);
    }
    else if (stage == 7)
    {
      // go back to home
      drillWayPointConstraints.push_back(home_constraints);
    }
    RCLCPP_INFO(LOGGER, "drillWayPointConstraints.size() %ld", drillWayPointConstraints.size());
    sendGlobalPlannerAction(true);
  }

  void HybridPlanningManager::drillCmd(bool start)
  {
    std_msgs::msg::String msg;
    if (start) {
      msg.data = "d";
    } else {
      msg.data = "s";
    }
    start_drilling_publisher_->publish(msg);
  }

}  // namespace moveit::hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit::hybrid_planning::HybridPlanningManager)
