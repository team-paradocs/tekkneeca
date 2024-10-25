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
// #include <moveit_cpp/planning_component.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <moveit/ompl_interface/ompl_interface.h>

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

  bool MoveItPlanningPipeline::initialize(const rclcpp::Node::SharedPtr &node)
  {

    // Planning Scene options
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "name", UNDEFINED);
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "robot_description", UNDEFINED);
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "joint_state_topic", UNDEFINED);
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "attached_collision_object_topic", UNDEFINED);
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "publish_planning_scene_topic", UNDEFINED);
    node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "monitored_planning_scene_topic", UNDEFINED);
    node->declare_parameter<double>(PLANNING_SCENE_MONITOR_NS + "wait_for_initial_state_timeout", 10.0);

    // Declare planning pipeline parameter
    node->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names",
                                                      std::vector<std::string>({UNDEFINED}));
    node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);

    // Declare planning pipeline OMPL parameters

    node->declare_parameter<std::vector<std::string>>("ompl.arm.planner_configs", std::vector<std::string>({"SBLkConfigDefault", "ESTkConfigDefault", "LBKPIECEkConfigDefault", "BKPIECEkConfigDefault", "KPIECEkConfigDefault",
                                                                                                            "RRTkConfigDefault", "RRTConnectkConfigDefault", "RRTstarkConfigDefault", "TRRTkConfigDefault", "PRMkConfigDefault",
                                                                                                            "PRMstarkConfigDefault", "FMTkConfigDefault", "BFMTkConfigDefault", "PDSTkConfigDefault", "STRIDEkConfigDefault",
                                                                                                            "BiTRRTkConfigDefault", "LBTRRTkConfigDefault", "BiESTkConfigDefault", "ProjESTkConfigDefault", "LazyPRMkConfigDefault",
                                                                                                            "LazyPRMstarkConfigDefault", "SPARSkConfigDefault", "SPARStwokConfigDefault", "TrajOptDefault"}));

    node->declare_parameter<std::string>("ompl.planner_configs.SBLkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.ESTkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.LBKPIECEkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.BKPIECEkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.KPIECEkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.RRTkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.RRTConnectkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.RRTstarkConfigDefault.type", UNDEFINED);
    node->declare_parameter<std::string>("ompl.planner_configs.TRRTkConfigDefault.type", "geometric::TRRT");
    node->declare_parameter<std::string>("ompl.planner_configs.PRMkConfigDefault.type", "geometric::PRM");
    node->declare_parameter<std::string>("ompl.planner_configs.PRMstarkConfigDefault.type", "geometric::PRMstar");
    node->declare_parameter<std::string>("ompl.planner_configs.FMTkConfigDefault.type", "geometric::FMT");
    node->declare_parameter<std::string>("ompl.planner_configs.BFMTkConfigDefault.type", "geometric::BFMT");
    node->declare_parameter<std::string>("ompl.planner_configs.PDSTkConfigDefault.type", "geometric::PDST");
    node->declare_parameter<std::string>("ompl.planner_configs.STRIDEkConfigDefault.type", "geometric::STRIDE");
    node->declare_parameter<std::string>("ompl.planner_configs.BiTRRTkConfigDefault.type", "geometric::BiTRRT");
    node->declare_parameter<std::string>("ompl.planner_configs.LBTRRTkConfigDefault.type", "geometric::LBTRRT");
    node->declare_parameter<std::string>("ompl.planner_configs.BiESTkConfigDefault.type", "geometric::BiEST");
    node->declare_parameter<std::string>("ompl.planner_configs.ProjESTkConfigDefault.type", "geometric::ProjEST");
    node->declare_parameter<std::string>("ompl.planner_configs.LazyPRMkConfigDefault.type", "geometric::LazyPRM");
    node->declare_parameter<std::string>("ompl.planner_configs.LazyPRMstarkConfigDefault.type", "geometric::LazyPRMstar");
    node->declare_parameter<std::string>("ompl.planner_configs.SPARSkConfigDefault.type", "geometric::SPARS");
    node->declare_parameter<std::string>("ompl.planner_configs.SPARStwokConfigDefault.type", "geometric::SPARStwo");
    node->declare_parameter<std::string>("ompl.planner_configs.TrajOptDefault.type", "geometric::TrajOpt");
    node->declare_parameter<std::string>("ompl.arm.projection_evaluator", "joints(A1, A2, A3, A4, A5, A6, A7)");

    // optional parameters
    // node->declare_parameter<int>("ompl.planner_configs.RRTstarkConfigDefault.delay_collision_checking", 1);
    // node->declare_parameter<double>("ompl.planner_configs.RRTstarkConfigDefault.goal_bias", 0.05);
    // node->declare_parameter<double>("ompl.planner_configs.RRTstarkConfigDefault.range", 0.0);

    node->declare_parameter<std::vector<std::string>>("ompl.planning_plugins", std::vector<std::string>({UNDEFINED}));
    node->declare_parameter<std::string>("ompl.planning_plugin", UNDEFINED);

    node->declare_parameter<std::string>("ompl.request_adapters", UNDEFINED);
    node->declare_parameter<std::string>("ompl.response_adapters", UNDEFINED);
    node->declare_parameter<double>("ompl.start_state_max_bounds_error", 0.1);

    // pilz_industrial_motion_planner parameters
    node->declare_parameter<std::string>("pilz_industrial_motion_planner.capabilities", UNDEFINED);
    node->declare_parameter<std::string>("pilz_industrial_motion_planner.default_planner_config", UNDEFINED);
    node->declare_parameter<std::vector<std::string>>("pilz_industrial_motion_planner.planning_plugins", std::vector<std::string>({UNDEFINED}));
    node->declare_parameter<std::string>("pilz_industrial_motion_planner.planning_plugin", UNDEFINED);
    node->declare_parameter<std::string>("pilz_industrial_motion_planner.request_adapters", UNDEFINED);
    node->declare_parameter<double>("pilz_industrial_motion_planner.cartesian_limits.max_trans_vel", 1.0);
    node->declare_parameter<double>("pilz_industrial_motion_planner.cartesian_limits.max_trans_acc", 2.25);
    node->declare_parameter<double>("pilz_industrial_motion_planner.cartesian_limits.max_trans_dec", -5.0);
    node->declare_parameter<double>("pilz_industrial_motion_planner.cartesian_limits.max_rot_vel", 1.57);

    // Default PlanRequestParameters. These can be overridden when plan() is called
    node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", UNDEFINED);
    node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", UNDEFINED);
    node->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", 5);
    node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", 1.0);
    node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", 1.0);
    node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", 1.0);

    // Trajectory Execution Functionality (required by the MoveItPlanningPipeline but not used within hybrid planning)
    node->declare_parameter<std::string>("moveit_controller_manager", UNDEFINED);
    // node->declare_parameter<bool>("allow_trajectory_execution", true);
    // node->declare_parameter<bool>("moveit_manage_controllers", true);

    node_ptr_ = node;

    // Initialize MoveItCpp API
    // The MoveItCpp here read in the parameters from the node
    // Using PLANNING_SCENE_MONITOR_NS, PLANNING_PIPELINES_NS
    moveit_cpp::MoveItCpp::Options moveit_cpp_options(node);
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);

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
    plan_params.planner_id = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planner_id").as_string();
    RCLCPP_INFO(LOGGER, "Shivangi Planner ID: %s", plan_params.planner_id.c_str());
    plan_params.planning_pipeline = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_pipeline").as_string();
    plan_params.planning_attempts = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_attempts").as_int();
    plan_params.planning_time = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "planning_time").as_double();
    RCLCPP_INFO(LOGGER, "Shivangi Planner time: %f", plan_params.planning_time);

    plan_params.max_velocity_scaling_factor = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor").as_double();
    plan_params.max_acceleration_scaling_factor = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor").as_double();

    // plan_params.cartesian_limits = node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits").as_string();
    double x_min, x_max, y_min, y_max, z_min, z_max;

    // Retrieve each parameter individually
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.x_min", x_min);
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.x_max", x_max);
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.y_min", y_min);
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.y_max", y_max);
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.z_min", z_min);
    node_ptr_->get_parameter(PLAN_REQUEST_PARAM_NS + "cartesian_limits.z_max", z_max);

    x_min = -100.0;
    x_max = 100.0;
    y_min = -100.0;
    y_max = 100.0;
    z_min = -100.0;
    z_max = 100.0;

    // Now you can use x_min, x_max, etc., as needed
    RCLCPP_INFO(LOGGER, "Cartesian Limits - x: [%f, %f], y: [%f, %f], z: [%f, %f]",
                x_min, x_max, y_min, y_max, z_min, z_max);

    // Create planning component
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(group_name, moveit_cpp_);

    moveit_msgs::msg::Constraints constraints;

    // Create position constraint for Cartesian bounds (loaded from YAML)
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = "link_0";
    position_constraint.link_name = "link_tool"; // Replace with the actual link name
    position_constraint.constraint_region.primitives.resize(1);
    position_constraint.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    position_constraint.constraint_region.primitives[0].dimensions = {x_max - x_min, y_max - y_min, z_max - z_min};

    // Set the pose of the box (the center point of the box)
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = (x_max + x_min) / 2.0; // Center of the box in X direction
    box_pose.position.y = (y_max + y_min) / 2.0; // Center of the box in Y direction
    box_pose.position.z = (z_max + z_min) / 2.0; // Center of the box in Z direction

    // Box orientation: Identity quaternion (no rotation)
    box_pose.orientation.w = 1.0;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;

    // Assign the pose to the box
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);

    // Apply position constraint to the path constraints
    constraints.position_constraints.push_back(position_constraint);

    // Set the path constraints in the PlanningComponent
    planning_components->setPathConstraints(constraints);

    // auto space = std::make_shared<ompl::base::SE3StateSpace>();
    // ompl::base::RealVectorBounds bounds(3);

    // // bounds.setLow(0, x_min);
    // bounds.setHigh(0, 100);
    // // bounds.setLow(1, y_min);
    // bounds.setHigh(1, 100);
    // // bounds.setLow(2, z_min);
    // bounds.setHigh(2, 100);

    // space->setBounds(bounds); // Apply the bounds to the state space

    // auto ompl_interface = std::make_shared<ompl_interface::OMPLInterface>(planning_components->getPlanningScene(), planning_components->getRobotModel());
    // ompl_interface->getPlanningContext()->getOMPLSimpleSetup()->getSpaceInformation()->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    planning_components->setStartStateToCurrentState();
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitor();
    auto planning_scene = planning_scene_monitor->getPlanningScene();
    auto current_state = planning_scene->getCurrentStateNonConst();
    // auto start_state = 

    // Ensure the current state is valid within the planning scene
    bool is_valid = planning_scene->isStateValid(current_state, planning_components->getPlanningGroupName());
    if (!is_valid)
    {
      // Print the Cartesian position of the current state
      const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Shivangi Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());
      RCLCPP_ERROR(rclcpp::get_logger("planner"), "Shivangi Start state is invalid!");
    }
    else
    {
      const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Shivangi Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());
      RCLCPP_INFO(rclcpp::get_logger("planner"), "Shivangi Start state is valid!");
    }
    // Copy goal constraint into planning component
    planning_components->setGoal(motion_plan_req.goal_constraints);

    // Plan motion
    auto plan_solution = planning_components->plan(plan_params);
    if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      response.error_code = plan_solution.error_code;
      return response;
    }

    // Transform solution into MotionPlanResponse and publish it
    response.trajectory_start = plan_solution.start_state;
    response.group_name = group_name;
    plan_solution.trajectory->getRobotTrajectoryMsg(response.trajectory);
    response.error_code = plan_solution.error_code;

    return response;
  }
} // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::MoveItPlanningPipeline, moveit::hybrid_planning::GlobalPlannerInterface);
