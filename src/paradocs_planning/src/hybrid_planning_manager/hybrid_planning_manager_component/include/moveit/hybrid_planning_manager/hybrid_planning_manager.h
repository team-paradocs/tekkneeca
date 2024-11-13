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

/* Author: Sebastian Jahr
   Description: The hybrid planning manager component node that serves as the control unit of the whole architecture.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/action/local_planner.hpp>
#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/hybrid_planning_manager/planner_logic_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <pluginlib/class_loader.hpp>

namespace moveit::hybrid_planning
{
  /**
  * Class HybridPlanningManager - ROS 2 component node that implements the hybrid planning manager.
  */
  class HybridPlanningManager : public rclcpp::Node
  {
  public:
    /** \brief Constructor */
    HybridPlanningManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .allow_undeclared_parameters(true));

    /** \brief Destructor */
    ~HybridPlanningManager() override
    {
      // Threading is removed
      // Join the thread used for long-running callbacks
      // if (long_callback_thread_.joinable())
      // {
      //   long_callback_thread_.join();
      // }
    }

    /**
    * Allows creation of a smart pointer that references to instances of this object
    * @return shared pointer of the HybridPlanningManager instance that called the function
    */
    std::shared_ptr<HybridPlanningManager> shared_from_this()
    {
      return std::static_pointer_cast<HybridPlanningManager>(Node::shared_from_this());
    }

    /**
    * Load and initialized planner logic plugin and ROS 2 action and topic interfaces
    * @return Initialization successful yes/no
    */
    bool initialize();

    void stopGlobalPlanner();
    
    void stopLocalPlanner();

    /**
    * Cancel any active action goals, including global and local planners
    */
    void cancelHybridManagerGoals() noexcept;

    /**
    * This handles execution of a hybrid planning goal in a new thread, to avoid blocking the executor.
    * @param goal_handle The action server goal
    */
    void executeHybridPlannerGoal(
        const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& goal_handle);

    /**
    * Send global planning request to global planner component
    * @return Global planner successfully started yes/no
    */
    bool sendGlobalPlannerAction(bool is_drill);

    /**
    * Send local planning request to local planner component
    * @return Local planner successfully started yes/no
    */
    bool sendLocalPlannerAction(int type);

    /**
    * Calculate IK
    */
    bool calculateIK();

    void drillMotion();

    const geometry_msgs::msg::PoseStamped computeOffsetPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& targetPose, float offset);

    std::shared_ptr<const geometry_msgs::msg::PoseStamped> 
      getHybridPlanningGoalHandle() const {
        return hybrid_planning_goal_handle_;
      }

  private:

    void declareParams()
    {
      const double UNDEFINEDDOUBLE = std::numeric_limits<double>::quiet_NaN();
      const int UNDEFINEDINT = -1;
      const std::string UNDEFINED = "<undefined>";

      this->declare_parameter<std::string>("planner_logic_plugin_name", UNDEFINED);
      this->declare_parameter<std::string>("local_planning_action_name", UNDEFINED);
      this->declare_parameter<std::string>("global_planning_action_name", UNDEFINED);

      // For IK calculation
      this->declare_parameter<std::string>("robot_description_kinematics.arm.kinematics_solver", UNDEFINED);
      this->declare_parameter<double>("robot_description_kinematics.arm.kinematics_solver_timeout", UNDEFINEDDOUBLE);
      this->declare_parameter<int>("robot_description_kinematics.arm.kinematics_solver_attempts", UNDEFINEDINT);
      this->declare_parameter<std::string>("robot_description_kinematics.arm.mode", UNDEFINED);
      this->declare_parameter<double>("robot_description_kinematics.arm.position_scale", UNDEFINEDDOUBLE); 
      this->declare_parameter<double>("robot_description_kinematics.arm.rotation_scale", UNDEFINEDDOUBLE);
      this->declare_parameter<double>("robot_description_kinematics.arm.position_threshold", UNDEFINEDDOUBLE);
      this->declare_parameter<double>("robot_description_kinematics.arm.orientation_threshold", UNDEFINEDDOUBLE);
      this->declare_parameter<double>("robot_description_kinematics.arm.cost_threshold", UNDEFINEDDOUBLE);
      this->declare_parameter<double>("robot_description_kinematics.arm.minimal_displacement_weight", UNDEFINEDDOUBLE);
      this->declare_parameter<double>("robot_description_kinematics.arm.gd_step_size", UNDEFINEDDOUBLE);

      this->declare_parameter<std::string>("robot_description_name", UNDEFINED);
      this->declare_parameter<std::string>("planning_group", UNDEFINED);
    }

    // Planner logic plugin loader
    std::unique_ptr<pluginlib::ClassLoader<PlannerLogicInterface>> planner_logic_plugin_loader_;

    // Planner logic instance to implement reactive behavior
    std::shared_ptr<PlannerLogicInterface> planner_logic_instance_;

    // Timer to trigger events periodically
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot model
    std::shared_ptr<moveit::core::RobotModel> robot_model_;

    // Planning group
    std::string planning_group_;

    // Goal from IK calculation
    std::shared_ptr<moveit::core::RobotState> goal_state_;

    std::shared_ptr<robot_trajectory::RobotTrajectory> cache_global_trajectory_;
    std::vector<moveit_msgs::msg::Constraints> drillWayPointConstraints;

    std::shared_ptr<const moveit::core::JointModelGroup> joint_model_group_;

    // Flag that indicates whether the manager is initialized
    bool initialized_;

    // Flag that indicates hybrid planning has been canceled
    std::atomic<int> planner_state_;
    std::atomic<bool> dill_started_ = false;

    // Shared hybrid planning goal handle
    std::shared_ptr<const geometry_msgs::msg::PoseStamped> hybrid_planning_goal_handle_;
    std::shared_ptr<const geometry_msgs::msg::PoseStamped> drill_pose_goal_handle_;

    // Planning request action clients
    rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SharedPtr local_planner_action_client_;
    rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planner_action_client_;

    // Stop execution publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_execution_publisher_;

    // Hybrid planning request action server
    rclcpp_action::Server<moveit_msgs::action::HybridPlanner>::SharedPtr hybrid_planning_request_server_;

    // Global solution subscriber
    rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_sub_;

    // Tracking goal subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_goal_sub_;

    // Drilling goal subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drilling_goal_sub_;

    // Planning state subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr planning_state_sub_;

    // This thread is used for long-running callbacks. It's a member so they do not go out of scope.
    // std::thread long_callback_thread_;

    // A unique callback group, to avoid mixing callbacks with other action servers
    // rclcpp::CallbackGroup::SharedPtr cb_group_;
  };
}  // namespace moveit::hybrid_planning
