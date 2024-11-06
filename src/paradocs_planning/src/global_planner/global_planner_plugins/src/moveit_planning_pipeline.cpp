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
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
// #include <moveit_cpp/planning_component.hpp>
#include <rclcpp/rclcpp.hpp>

// import movegroupinterface
//  #include <moveit/move_group_interface/move_group_interface.h>

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
    node->declare_parameter<bool>("ompl.arm.enforce_constrained_state_space", true);

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

    // auto moveit_visual_tools =
    //     moveit_visual_tools::MoveItVisualTools{node, "link_0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    //                                            moveit_cpp_->getRobotModel()};
    rclcpp::sleep_for(std::chrono::seconds(1));
    // moveit_visual_tools.deleteAllMarkers();
    // moveit_visual_tools.loadRemoteControl();
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

    // Create planning component
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(group_name, moveit_cpp_);

    RCLCPP_INFO(LOGGER, "Shivangi here");

    moveit_msgs::msg::Constraints constraints;

    // Create position constraint for Cartesian bounds (loaded from YAML)
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = "link_0";
    // position_constraint.header.frame_id =
    position_constraint.link_name = "link_tool"; // Replace with the actual link name
    // position_constraint.constraint_region.primitives.resize(3);

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.7, 0.8, 0.8};
    // position_constraint.constraint_region.primitives[0].dimensions = {0, 0, 0};
    // position_constraint.constraint_region.primitives[0].dimensions = {x_max - x_min, y_max - y_min, z_max - z_min};

    // Set the pose of the box (the center point of the box)
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.5; // Center of the box in X direction
    box_pose.position.y = 0; // Center of the box in Y direction
    box_pose.position.z = 0.4; // Center of the box in Z direction

    // Box orientation: Identity quaternion (no rotation)
    box_pose.orientation.w = 1.0;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;

    // Assign the pose to the box
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitor();
    auto planning_scene = planning_scene_monitor->getPlanningScene();

    //////CREATING SMALLER BOX CONSTRAINT
    planning_components->setStartStateToCurrentState();
    auto current_state = planning_scene->getCurrentStateNonConst();
    const auto &link_state = current_state.getGlobalLinkTransform("link_tool"); // Replace "link_tool" with the actual link name
      RCLCPP_INFO(LOGGER, "Shivangi Current Cartesian Position - x: %f, y: %f, z: %f",
                  link_state.translation().x(), link_state.translation().y(), link_state.translation().z());
      

    shape_msgs::msg::SolidPrimitive small_box;
    small_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    geometry_msgs::msg::Pose small_box_pose;

    if (!motion_plan_req.goal_constraints.empty())
    {
      const auto &goal_constraints = motion_plan_req.goal_constraints[0];
      if (!goal_constraints.position_constraints.empty())
      {
        const auto &goal_position = goal_constraints.position_constraints[0].constraint_region.primitive_poses[0].position;
        RCLCPP_INFO(LOGGER, "Goal Position - x: %f, y: %f, z: %f", goal_position.x, goal_position.y, goal_position.z);
        double ee_size = 2*0.05;
        double x_dim = std::abs(goal_position.x - link_state.translation().x());
        double y_dim = std::abs(goal_position.y - link_state.translation().y());
        double z_dim = 1.0+std::abs(goal_position.z - link_state.translation().z());
        small_box.dimensions = {x_dim + ee_size, y_dim + ee_size, z_dim + ee_size};
        // small_box.dimensions = {std::abs(goal_position.x - link_state.translation().x()), 
        //       std::abs(goal_position.y - link_state.translation().y()), 
        //       std::abs(goal_position.z - link_state.translation().z())};
              small_box_pose.position.x = (goal_position.x + link_state.translation().x()) / 2;
              small_box_pose.position.y = (goal_position.y + link_state.translation().y()) / 2;
              small_box_pose.position.z = (goal_position.z + link_state.translation().z()) / 2;
      }
      if (!goal_constraints.orientation_constraints.empty())
      {
        const auto &goal_orientation = goal_constraints.orientation_constraints[0].orientation;
        RCLCPP_INFO(LOGGER, "Goal Orientation - w: %f, x: %f, y: %f, z: %f", goal_orientation.w, goal_orientation.x, goal_orientation.y, goal_orientation.z);
      }
    }

    // position_constraint.constraint_region.primitives.emplace_back(small_box);
    // position_constraint.constraint_region.primitives[0].dimensions = {0, 0, 0};
    // position_constraint.constraint_region.primitives[0].dimensions = {x_max - x_min, y_max - y_min, z_max - z_min};

    // Set the pose of the small_box (the center point of the small_box)
    
    // small_box_pose.position.x = -0.5; // Center of the small_box in X direction
    // small_box_pose.position.y = 0; // Center of the small_box in Y direction
    // small_box_pose.position.z = 0.4; // Center of the small_box in Z direction

    // small_Box orientation: Identity quaternion (no rotation)
    small_box_pose.orientation.w = 1.0;
    small_box_pose.orientation.x = 0.0;
    small_box_pose.orientation.y = 0.0;
    small_box_pose.orientation.z = 0.0;

    // Assign the pose to the small_box
    // position_constraint.constraint_region.primitive_poses.emplace_back(small_box_pose);
    position_constraint.constraint_region.primitives.emplace_back(box);
    position_constraint.constraint_region.primitive_poses.emplace_back(box_pose);



    // Extract the position and orientation from the goal constraints
    

    // Create orientation constraint
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = "link_0";
    orientation_constraint.link_name = "link_tool"; // Replace with the actual link name
    // Get the current orientation of the end effector
    const auto &current_orientation = link_state.rotation();
    Eigen::Quaterniond current_quat(current_orientation);

    // Set the orientation constraint to the current orientation
    orientation_constraint.orientation.w = current_quat.w();
    orientation_constraint.orientation.x = current_quat.x();
    orientation_constraint.orientation.y = current_quat.y();
    orientation_constraint.orientation.z = current_quat.z();
    orientation_constraint.absolute_x_axis_tolerance = 0.4; // Tolerance values
    orientation_constraint.absolute_y_axis_tolerance = 0.4;
    orientation_constraint.absolute_z_axis_tolerance = 0.4;
    orientation_constraint.weight = 0.5; // Weight of the constraint

    constraints.orientation_constraints.push_back(orientation_constraint); // TOGGLE ORIENTATION CONSTRAINT

    constraints.position_constraints.emplace_back(position_constraint); // TOGGLE POSITION CONSTRAINT

    // Set the path constraints in the PlanningComponent
    planning_components->setPathConstraints(constraints); // TOGGLE CONSTRAINT

    // moveit_visual_tools::MoveItVisualTools moveit_visual_tools("link_0");

    Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                                box_pose.position.z - box.dimensions[2] / 2);
    Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                                box_pose.position.z + box.dimensions[2] / 2);
    // moveit_visual_tools.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
    // moveit_visual_tools.trigger();

    RCLCPP_INFO(LOGGER, "Shivangi Path Constraints: %s", constraints.position_constraints[0].link_name.c_str());
    // RCLCPP_INFO(LOGGER, "Shivangi setting workspace");
    planning_components->setWorkspace(-1, -0.5, 0, 0, 0.5, 1);

    // // get the path constraints and print them
    // auto path_constraints = planning_components->getPathConstraints();
    // RCLCPP_INFO(LOGGER, "Shivangi Path Constraints: %s", path_constraints.position_constraints[0].link_name.c_str());

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

    
    
    
    // planning_components->setStartState(current_state);
    // auto start_state =

    // Ensure the current state is valid within the planning scene
    bool is_valid = planning_scene->isStateValid(current_state, planning_components->getPlanningGroupName());
    RCLCPP_INFO(LOGGER, "Shivangi State validity: %s", is_valid ? "valid" : "invalid");
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
    // if (!motion_plan_req.goal_constraints.empty())
    // {
    //   for (auto &goal_constraint : motion_plan_req.goal_constraints)
    //   {
    //     for (auto &orientation_constraint : goal_constraint.orientation_constraints)
    //     {
    //       orientation_constraint.orientation.w = current_quat.w();
    //       orientation_constraint.orientation.x = current_quat.x();
    //       orientation_constraint.orientation.y = current_quat.y();
    //       orientation_constraint.orientation.z = current_quat.z();
    //     }
    //   }
    // }
    planning_components->setGoal(motion_plan_req.goal_constraints);

    // Plan motion
    auto plan_solution = planning_components->plan(plan_params);
    int max_attempts = 15;
    int attempts = 0;
    while (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS && attempts < max_attempts)
    {
      RCLCPP_WARN(LOGGER, "Planning attempt %d failed with error code: %d. Retrying...", attempts + 1, plan_solution.error_code.val);
      plan_solution = planning_components->plan(plan_params);
      attempts++;
    }

    if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      response.error_code = plan_solution.error_code;
      RCLCPP_ERROR(LOGGER, "Planning failed after %d attempts with error code: %d", attempts, plan_solution.error_code.val);
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
