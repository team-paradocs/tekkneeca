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

/* Author: Sebastian Jahr
   Description: A global planner component node that is customizable through plugins.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pluginlib/class_loader.hpp>

#include <moveit/global_planner/global_planner_interface.h>

#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit::hybrid_planning
{
// Component node containing the global planner
class GlobalPlannerComponent
{
public:
  /** \brief Constructor */
  GlobalPlannerComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .allow_undeclared_parameters(true));

  /** \brief Destructor */
  ~GlobalPlannerComponent()
  {
    // Join the thread used for long-running callbacks
    if (long_callback_thread_.joinable())
    {
      long_callback_thread_.join();
    }
  }

  // This function is required to make this class a valid NodeClass
  // see https://docs.ros2.org/foxy/api/rclcpp_components/register__node__macro_8hpp.html
  // Skip linting due to unconventional function naming
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()  // NOLINT
  {
    return node_->get_node_base_interface();  // NOLINT
  }

private:

  void declareParams(const rclcpp::Node::SharedPtr& n) {
    const std::string PLANNING_SCENE_MONITOR_NS = "planning_scene_monitor_options.";
    const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
    const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
    const double UNDEFINEDDOUBLE = std::numeric_limits<double>::quiet_NaN();
    const int UNDEFINEDINT = -1;
    const std::string UNDEFINED = "<undefined>";

    n->declare_parameter<std::string>("global_planning_action_name", "");
    n->declare_parameter<std::string>("global_planner_name", UNDEFINED);

    // Planning Scene options
    {
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "name", UNDEFINED);
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "robot_description", UNDEFINED);
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "joint_state_topic", UNDEFINED);
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "attached_collision_object_topic", UNDEFINED);
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "publish_planning_scene_topic", UNDEFINED);
      n->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "monitored_planning_scene_topic", UNDEFINED);
      n->declare_parameter<double>(PLANNING_SCENE_MONITOR_NS + "wait_for_initial_state_timeout", UNDEFINEDDOUBLE);
    }

    // Declare planning pipeline parameter
    {
      n->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names", std::vector<std::string>({UNDEFINED}));
      n->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);
    }

    // Declare planning pipeline OMPL parameters
    {
      n->declare_parameter<std::vector<std::string>>("ompl.arm.planner_configs", std::vector<std::string>({UNDEFINED}));
      n->declare_parameter<std::string>("ompl.planner_configs.SBLkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.ESTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.LBKPIECEkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.BKPIECEkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.KPIECEkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.RRTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.RRTConnectkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.RRTstarkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.TRRTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.PRMkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.PRMstarkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.FMTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.BFMTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.PDSTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.STRIDEkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.BiTRRTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.LBTRRTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.BiESTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.ProjESTkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.LazyPRMkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.LazyPRMstarkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.SPARSkConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.SPARStwokConfigDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.planner_configs.TrajOptDefault.type", UNDEFINED);
      n->declare_parameter<std::string>("ompl.arm.projection_evaluator", UNDEFINED);
      n->declare_parameter<bool>("ompl.arm.enforce_constrained_state_space", true);

      n->declare_parameter<std::vector<std::string>>("ompl.planning_plugins", std::vector<std::string>({UNDEFINED}));
      n->declare_parameter<std::string>("ompl.planning_plugin", UNDEFINED);

      n->declare_parameter<std::string>("ompl.request_adapters", UNDEFINED);
      n->declare_parameter<std::string>("ompl.response_adapters", UNDEFINED);
      n->declare_parameter<double>("ompl.start_state_max_bounds_error", UNDEFINEDDOUBLE);
    }
    
    // Declare planning pipeline Pilz parameters
    {
      n->declare_parameter<std::string>("pilz_industrial_motion_planner.capabilities", UNDEFINED);
      n->declare_parameter<std::string>("pilz_industrial_motion_planner.default_planner_config", UNDEFINED);
      n->declare_parameter<std::vector<std::string>>("pilz_industrial_motion_planner.planning_plugins", std::vector<std::string>({UNDEFINED}));
      n->declare_parameter<std::string>("pilz_industrial_motion_planner.planning_plugin", UNDEFINED);
      n->declare_parameter<std::string>("pilz_industrial_motion_planner.request_adapters", UNDEFINED);
      n->declare_parameter<std::string>("pilz_industrial_motion_planner.response_adapters", UNDEFINED);
      n->declare_parameter<double>("pilz_industrial_motion_planner.start_state_max_bounds_error", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.cartesian_limits.max_trans_vel", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.cartesian_limits.max_trans_acc", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.cartesian_limits.max_trans_dec", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.cartesian_limits.max_rot_vel", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.default_acceleration_scaling_factor", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_planning.default_velocity_scaling_factor", UNDEFINEDDOUBLE);

      struct JointLimits 
      {
        std::string name;
        bool angle_wraparound;
        bool has_acceleration_limits;
        bool has_deceleration_limits;
        bool has_effort_limits;
        bool has_position_limits;
        bool has_soft_limits;
        bool has_velocity_limits;
        double k_position;
        double k_velocity;
        double max_acceleration;
        double max_deceleration;
        double max_effort;
        double max_jerk;
        double max_position;
        double max_velocity;
        double min_position;
        double min_velocity;
        double soft_lower_limit;
        double soft_upper_limit;
      };

      std::vector<JointLimits> joint_limits = 
      {
          {"A1", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A2", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A3", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A4", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A5", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A6", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE},
          {"A7", false, true, true, true, true, false, true, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE, UNDEFINEDDOUBLE}
      };

      for (const auto& joint : joint_limits) 
      {
          std::string prefix = "robot_description_planning.joint_limits." + joint.name + ".";
          n->declare_parameter<bool>(prefix + "angle_wraparound", joint.angle_wraparound);
          n->declare_parameter<bool>(prefix + "has_acceleration_limits", joint.has_acceleration_limits);
          n->declare_parameter<bool>(prefix + "has_deceleration_limits", joint.has_deceleration_limits);
          n->declare_parameter<bool>(prefix + "has_effort_limits", joint.has_effort_limits);
          n->declare_parameter<bool>(prefix + "has_position_limits", joint.has_position_limits);
          n->declare_parameter<bool>(prefix + "has_soft_limits", joint.has_soft_limits);
          n->declare_parameter<bool>(prefix + "has_velocity_limits", joint.has_velocity_limits);
          n->declare_parameter<double>(prefix + "k_position", joint.k_position);
          n->declare_parameter<double>(prefix + "k_velocity", joint.k_velocity);
          n->declare_parameter<double>(prefix + "max_acceleration", joint.max_acceleration);
          n->declare_parameter<double>(prefix + "max_deceleration", joint.max_deceleration);
          n->declare_parameter<double>(prefix + "max_effort", joint.max_effort);
          n->declare_parameter<double>(prefix + "max_jerk", joint.max_jerk);
          n->declare_parameter<double>(prefix + "max_position", joint.max_position);
          n->declare_parameter<double>(prefix + "max_velocity", joint.max_velocity);
          n->declare_parameter<double>(prefix + "min_position", joint.min_position);
          n->declare_parameter<double>(prefix + "min_velocity", joint.min_velocity);
          n->declare_parameter<double>(prefix + "soft_lower_limit", joint.soft_lower_limit);
          n->declare_parameter<double>(prefix + "soft_upper_limit", joint.soft_upper_limit);
      }

    }

    // Declare planning request parameters
    {
      n->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", UNDEFINED);
      n->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", UNDEFINED);
      n->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", UNDEFINEDINT);
      n->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", UNDEFINEDDOUBLE);
      n->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", UNDEFINEDDOUBLE);
      n->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", UNDEFINEDDOUBLE);
    }

    // For IK calculation
    {
      n->declare_parameter<std::string>("robot_description_kinematics.arm.kinematics_solver", UNDEFINED);
      n->declare_parameter<double>("robot_description_kinematics.arm.kinematics_solver_timeout", UNDEFINEDDOUBLE);
      n->declare_parameter<int>("robot_description_kinematics.arm.kinematics_solver_attempts", UNDEFINEDINT);
      n->declare_parameter<std::string>("robot_description_kinematics.arm.mode", UNDEFINED);
      n->declare_parameter<double>("robot_description_kinematics.arm.position_scale", UNDEFINEDDOUBLE); 
      n->declare_parameter<double>("robot_description_kinematics.arm.rotation_scale", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_kinematics.arm.position_threshold", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_kinematics.arm.orientation_threshold", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_kinematics.arm.cost_threshold", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_kinematics.arm.minimal_displacement_weight", UNDEFINEDDOUBLE);
      n->declare_parameter<double>("robot_description_kinematics.arm.gd_step_size", UNDEFINEDDOUBLE);   
    }

    // MoveIt controller manager
    {
      n->declare_parameter<std::string>("moveit_controller_manager", UNDEFINED);
      n->declare_parameter<bool>("moveit_manage_controllers", true);
      // Declare controller-specific parameters
      n->declare_parameter<std::vector<std::string>>("moveit_simple_controller_manager.controller_names", {UNDEFINED});

      n->declare_parameter<std::string>("moveit_simple_controller_manager.joint_trajectory_controller.action_ns", UNDEFINED);
      n->declare_parameter<bool>("moveit_simple_controller_manager.joint_trajectory_controller.default", true);
      n->declare_parameter<std::vector<std::string>>("moveit_simple_controller_manager.joint_trajectory_controller.joints", 
                                                    {UNDEFINED});
      n->declare_parameter<std::string>("moveit_simple_controller_manager.joint_trajectory_controller.type", UNDEFINED);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;

  std::string planner_plugin_name_;

  // Global planner plugin loader
  std::unique_ptr<pluginlib::ClassLoader<GlobalPlannerInterface>> global_planner_plugin_loader_;

  // Global planner instance
  std::shared_ptr<GlobalPlannerInterface> global_planner_instance_;

  // Global planning request action server
  rclcpp_action::Server<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planning_request_server_;

  // Global trajectory publisher
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_pub_;

  // Goal callback for global planning request action server
  void globalPlanningRequestCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& goal_handle);

  // Initialize planning scene monitor and load pipelines
  bool initializeGlobalPlanner();

  // Flag to avoid publishing the global plan
  std::atomic<bool> dont_pub_;

  // This thread is used for long-running callbacks. It's a member so they do not go out of scope.
  std::thread long_callback_thread_;

  // A unique callback group, to avoid mixing callbacks with other action servers
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace moveit::hybrid_planning
