from typing import Dict, List, Optional, Union

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class GazeboMixin:
    @staticmethod
    def include_gazebo(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            **kwargs,
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> Node:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name")
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                LaunchConfiguration("robot_name"),
                "-robot_namespace",
                LaunchConfiguration("robot_name"),
                "--ros-args",
                "--log-level",
                "warn",
            ],
            output="screen",
            namespace=LaunchConfiguration("robot_name"),
            **kwargs,
        )


class LBRDescriptionMixin:
    @staticmethod
    def param_robot_description(
        model: Optional[Union[LaunchConfiguration, str]] = None,
        robot_name: Optional[Union[LaunchConfiguration, str]] = None,
        port_id: Optional[Union[LaunchConfiguration, str]] = None,
        sim: Optional[Union[LaunchConfiguration, bool]] = None,
    ) -> Dict[str, str]:
        if model is None:
            model = LaunchConfiguration("model", default="iiwa7")
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name", default="lbr")
        if port_id is None:
            port_id = LaunchConfiguration("port_id", default="30200")
        if sim is None:
            sim = LaunchConfiguration("sim", default="true")
        if type(sim) is bool:
            sim = "true" if sim else "false"
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_description"),
                            "urdf",
                            model,
                            model,
                        ]
                    ),
                    ".urdf.xacro",
                    " robot_name:=",
                    robot_name,
                    " port_id:=",
                    port_id,
                    " sim:=",
                    sim,
                    " initial_positions_file:=",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_description"),
                            "urdf",
                            model,
                            "initial_positions.yaml",
                        ]
                    ),
                ]
            )
        }
        return robot_description

    @staticmethod
    def arg_model(default_value: str = "iiwa7") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value=default_value,
            description="The LBR model in use.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )

    @staticmethod
    def arg_robot_name(default_value: str = "lbr") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value=default_value,
            description="The robot's name.",
        )

    @staticmethod
    def arg_port_id(default_value: str = "30200") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="port_id",
            default_value=default_value,
            description="Port ID of the FRI communication. Valid in range [30200, 30209].\n"
            "\tUsefull in multi-robot setups.",
        )

    @staticmethod
    def arg_sim(default_value: str = "true") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value=default_value,
            description="Whether to use the simulation or not.",
        )

    @staticmethod
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name", default="lbr")}

    @staticmethod
    def param_port_id() -> Dict[str, LaunchConfiguration]:
        return {"port_id": LaunchConfiguration("port_id", default="30200")}

    @staticmethod
    def param_sim() -> Dict[str, LaunchConfiguration]:
        return {"sim": LaunchConfiguration("sim", default="true")}

    @staticmethod
    def node_static_tf(
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        parent: Optional[Union[LaunchConfiguration, str]] = None,
        child: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs,
    ) -> Node:
        label = ["--x", "--y", "--z", "--roll", "--pitch", "--yaw"]
        tf = [str(x) for x in tf]
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[item for pair in zip(label, tf) for item in pair]
            + [
                "--frame-id",
                parent,
                "--child-frame-id",
                child,
                "--ros-args",
                "--log-level",
                "warn",
            ],
            **kwargs,
        )


class RVizMixin:
    @staticmethod
    def arg_rviz_config_pkg(
        default_value: str = "lbr_description",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config(
        default_value: str = "config/config.rviz",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_pkg: Optional[Union[LaunchConfiguration, str]] = None,
        rviz_config: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs,
    ) -> Node:
        if rviz_config_pkg is None:
            rviz_config_pkg = LaunchConfiguration(
                "rviz_config_pkg", default="lbr_description"
            )
        if rviz_config is None:
            rviz_config = LaunchConfiguration(
                "rviz_config", default="config/config.rviz"
            )
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(rviz_config_pkg),
                        rviz_config,
                    ]
                ),
                "--ros-args",
                "--log-level",
                "warn",
            ],
            **kwargs,
        )
