from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

from lbr_bringup import LBRMoveGroupMixin
from lbr_description import GazeboMixin, LBRDescriptionMixin, RVizMixin
from lbr_ros2_control import LBRROS2ControlMixin

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:

    ld = LaunchDescription()

    robot_description = LBRDescriptionMixin.param_robot_description(sim=True)

    robot_description_semantic_config = load_file(
        "med7_moveit_config", "config/med7.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Gazebo has its own controller manager
    ld.add_action(GazeboMixin.include_gazebo())
    spawn_entity = GazeboMixin.node_spawn_entity()
    ld.add_action(spawn_entity)
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    ld.add_action(joint_state_broadcaster)
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    )
    ld.add_action(
        robot_state_publisher
    )  # Do not condition robot state publisher on joint state broadcaster as Gazebo uses robot state publisher to retrieve robot description
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
    )

    # MoveIt 2
    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())

    # MoveGroup:
    # - requires world frame
    # - urdf only has robot_name/world
    # This transform needs publishing
    robot_name = LaunchConfiguration("robot_name").perform(context)
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=[0, 0, 0, 0, 0, 0],  # keep zero
            parent="world",
            child=PathJoinSubstitution(
                [
                    robot_name,
                    "world",
                ]  # results in robot_name/world
            ),
            parameters=[{"use_sim_time": True}],
        )
    )


    # Load additional OMPL pipeline
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugins": [
                "ompl_interface/OMPLPlanner",
            ],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    ompl_planning_yaml = load_yaml(
        "med7_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    model = LaunchConfiguration("model").perform(context)
    moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )
    movegroup_params = LBRMoveGroupMixin.params_move_group()

    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                ompl_planning_pipeline_config,
                movegroup_params,
                {"use_sim_time": True},
            ],
            condition=IfCondition(LaunchConfiguration("moveit")),
            namespace=robot_name,
        )
    )

    # RViz and MoveIt
    rviz_moveit = RVizMixin.node_rviz(
        rviz_config_pkg=f"{model}_moveit_config",
        rviz_config="config/moveit.rviz",
        parameters=LBRMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        )
        + [{"use_sim_time": True}],
        condition=IfCondition(
            AndSubstitution(LaunchConfiguration("moveit"), LaunchConfiguration("rviz"))
        ),
        remappings=[
            ("display_planned_path", robot_name + "/display_planned_path"),
            ("joint_states", robot_name + "/joint_states"),
            ("monitored_planning_scene", robot_name + "/monitored_planning_scene"),
            ("planning_scene", robot_name + "/planning_scene"),
            ("planning_scene_world", robot_name + "/planning_scene_world"),
            ("robot_description", robot_name + "/robot_description"),
            ("robot_description_semantic", robot_name + "/robot_description_semantic"),
        ],
    )

    # RViz no MoveIt
    rviz = RVizMixin.node_rviz(
        rviz_config_pkg="lbr_bringup",
        rviz_config="config/config.rviz",
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("rviz"),
                NotSubstitution(LaunchConfiguration("moveit")),
            )
        ),
    )

    # RViz event handler
    rviz_event_handler = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[rviz_moveit, rviz])
    )
    ld.add_action(rviz_event_handler)

    #  ========= hybrid_planning part =========

    kinematics_yaml = load_yaml(
        "med7_moveit_config", "config/kinematics.yaml"
    )

    moveit_simple_controllers_yaml = load_yaml(
        "med7_moveit_config", "config/moveit_controllers.yaml"
    )

    # lbr_controllers.yaml includes moveit_controllers.yaml
    # TODO: might need to refactor to lbr namespace
    # moveit_simple_controllers_yaml = load_yaml(
    #     "lbr_ros2_control", "config/lbr_controllers.yaml"
    # )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Any parameters that are unique to your plugins go here
    common_hybrid_planning_param = load_yaml(
        "paradocs_planning", "config/common_hybrid_planning_params.yaml"
    )
    global_planner_param = load_yaml(
        "paradocs_planning", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "paradocs_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "paradocs_planning", "config/hybrid_planning_manager.yaml"
    )

    # Generate launch description with multiple components
    # TODO: might need to remap to lbr namespace
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/lbr",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="paradocs_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                ],
                remappings=[
                    ("lbr/attached_collision_object", "/attached_collision_object"),
                    ("lbr/joint_states", "/joint_states"),
                    ("lbr/monitored_planning_scene", "/monitored_planning_scene"),
                    ("lbr/planning_scene", "/planning_scene"),
                    ("/display_planned_path", "/lbr/display_planned_path"),
                    ("/display_contacts", "/lbr/display_contacts"),
                    ("/trajectory_execution_event", "/lbr/trajectory_execution_event"),
                ],
            ),
            ComposableNode(
                package="paradocs_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                ],
                remappings=[
                    ("/joint_trajectory_controller/joint_trajectory", "/lbr/joint_trajectory_controller/joint_trajectory"),
                    ("lbr/joint_states", "/joint_states"),
                    ("lbr/collision_object", "/collision_object"),
                    ("lbr/planning_scene", "/planning_scene"),
                ],
            ),
            ComposableNode(
                package="paradocs_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                ],
            ),
        ],
        output="screen",
    )

    ld.add_action(container)

    demo_node = Node(
        package="paradocs_planning",
        executable="hybrid_planning_demo",
        name="hybrid_planning_demo_node",
        namespace="",
        output="screen",
        parameters=[
            # for gazebo
            {'use_sim_time': True},
            robot_description,
            robot_description_semantic,
            common_hybrid_planning_param,
        ],
        remappings=[
            ("/joint_states", "/lbr/joint_states"),
            ("/planning_scene", "/lbr/planning_scene"),
        ],
    )

    ld.add_action(demo_node)    

    # kuka_joint_group_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "kuka_joint_group_position_controller",
    #         "-c",
    #         "/controller_manager",
    #     ],
    # )

    # ld.add_action(kuka_joint_group_position_controller_spawner)

    return ld.entities


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
    # ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(DeclareLaunchArgument(
            name="model",
            default_value="med7",
            description="Robot model.",
        )
    )
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(
        DeclareLaunchArgument(
            name="moveit",
            default_value="true",
            description="Whether to launch MoveIt 2.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="rviz", default_value="true", description="Whether to launch RViz."
        )
    )
    # Gazebo loads controller configuration through lbr_description/gazebo/*.xacro from lbr_ros2_control/config/lbr_controllers.yaml
    ld.add_action(
        LBRROS2ControlMixin.arg_ctrl()
    )
    
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
