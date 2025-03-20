from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch_ros.actions import (
    Node,
)

from launch.substitutions import (
    PathJoinSubstitution,
)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
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
                        "tekkneeca.launch.py",
                    ]
                )
            ),
        )
    )

    # for planning scene
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    
    robot_description_semantic = {
        "robot_description_semantic": load_file("med7_moveit_config", "config/med7.srdf")
    }

    ld.add_action(
        Node(
            package="paradocs_planning",
            executable="robot_motion_planning",
            name="robot_motion_planning",
            namespace="/lbr",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("paradocs_planning"),
                        "config",
                        "robot_motion_planning.yaml",
                    ]
                ),
                robot_description,
                robot_description_semantic
            ],
            remappings=[
                    ("lbr/attached_collision_object", "/attached_collision_object"),
                    ("lbr/joint_states", "/joint_states"),
                    ("lbr/monitored_planning_scene", "/monitored_planning_scene"),
                    ("lbr/planning_scene", "/planning_scene"),
                    ("/display_planned_path", "/lbr/display_planned_path"),
                    ("/display_contacts", "/lbr/display_contacts"),
                    ("/trajectory_execution_event", "/lbr/trajectory_execution_event"),
                    ("/joint_trajectory_controller/joint_trajectory", "/lbr/joint_trajectory_controller/joint_trajectory"),
                    ("lbr/collision_object", "/collision_object"),
                    ("/lbr/tracked_pose", "/tracked_pose"),
            ],
        )
    )

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
    
    ld.add_action(
        LBRROS2ControlMixin.arg_ctrl()
    )
    
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
