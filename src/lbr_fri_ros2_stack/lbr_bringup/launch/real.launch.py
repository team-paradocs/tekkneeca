from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

from lbr_bringup import LBRMoveGroupMixin
from lbr_description import LBRDescriptionMixin, RVizMixin
from lbr_ros2_control import LBRROS2ControlMixin


import os
import yaml
from ament_index_python.packages import get_package_share_directory

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

    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    # need to specify the ros2_control node, load lbr_controllers.yaml, which specify the param for controllers
    # ctrl_cfg
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # joint state broad caster and controller on ros2 control node start
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    # not sure about the force_torque sensor is on every joint or just ee
    force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="force_torque_broadcaster"
    )
    # aside from robot_state_publisher, we also need lbr_state_broadcaster
    lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_state_broadcaster"
    )
    # spawn controllers according to lbr_controllers.yaml through controller manager
    # ctrl
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                force_torque_broadcaster,
                lbr_state_broadcaster,
                controller,
            ],
        )
    )
    ld.add_action(controller_event_handler)

    # robot state publisher on joint state broadcaster spawn exit
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    robot_state_publisher_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[robot_state_publisher]
        )
    )
    ld.add_action(robot_state_publisher_event_handler)

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
            # don't use sim time
        )
    )


    # Load additional OMPL pipeline
    ompl_planning_pipeline_config = {
        "ompl_2": {
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
    ompl_planning_pipeline_config["ompl_2"].update(ompl_planning_yaml)

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
                {"use_sim_time": False},
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
        + [{"use_sim_time": False}],
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
        OnProcessStart(
            target_action=robot_state_publisher, on_start=[rviz_moveit, rviz]
        )
    )
    ld.add_action(rviz_event_handler)

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
    ld.add_action(LBRDescriptionMixin.arg_port_id())
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
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
