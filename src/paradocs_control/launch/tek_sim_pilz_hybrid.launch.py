from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch_ros.actions import (
    ComposableNodeContainer
)

from launch.substitutions import (
    PathJoinSubstitution,
)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from lbr_description import LBRDescriptionMixin
from lbr_ros2_control import LBRROS2ControlMixin


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

    ld.add_action(LBRDescriptionMixin.arg_sim())

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("paradocs_control"),
                        "launch",
                        "tekkneeca_sim.launch.py",
                    ]
                )
            ),
        )
    )

    #  ========= hybrid_planning part =========

    robot_description = LBRDescriptionMixin.param_robot_description(sim=True)
    
    robot_description_semantic = {
        "robot_description_semantic": load_file("med7_moveit_config", "config/med7.srdf")
    }
    
    # Any parameters that are unique to your plugins go here
    common_hybrid_planning_param = load_yaml(
        "paradocs_planning", "config/common_hybrid_planning_params.yaml"
    )
    global_planner_param = load_yaml(
        "paradocs_planning", "config/global_planner_pilz.yaml"
    )
    local_planner_param = load_yaml(
        "paradocs_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "paradocs_planning", "config/hybrid_planning_manager.yaml"
    )

    # Generate launch description with multiple components
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
                    robot_description,
                ],
                # extra_arguments=[
                #     {"use_intra_process_comms": True},
                #     {'automatically_declare_parameters_from_overrides': True},
                #     {'allow_undeclared_parameters': True}
                # ],
            ),
        ],
        output="screen",
    )

    ld.add_action(container)


    return ld.entities


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()
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
