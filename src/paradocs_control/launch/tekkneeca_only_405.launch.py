from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnIncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from lbr_description import LBRDescriptionMixin, RVizMixin

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "real.launch.py",
                    ]
                )
            ),
        )
    )


    arg_name = DeclareLaunchArgument('name',             
                default_value=PathJoinSubstitution([
                FindPackageShare('paradocs_control'),  # Finds the install/share directory for your package
                TextSubstitution(text='config/eih_cam1')  # Appends the relative path to your file
            ]),)

    handeye_publisher = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher', parameters=[{
        'name': LaunchConfiguration('name'),
    }])

    ld.add_action(arg_name)
    ld.add_action(handeye_publisher)
 

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("paradocs_control"),
                        "launch",
                        "static_obstacles.launch.py",
                    ]
                )
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("paradocs_control"),
                        "launch",
                        "rs_dual_camera_launch.py",
                    ]
                )
            ),
            launch_arguments={
                'serial_no1': '_128422270653',
                # 'serial_no2': '_242322073889',
                'camera_name1': 'camera',
                # 'camera_name2': 'D435',
                'camera_namespace1': '',
                # 'camera_namespace2': '',
            }.items(),
        )
    )

    # now testing aruco_pose, uncomment when doing the drill_pose
    drill_pose_transformer = Node(package='paradocs_control', executable='drill_pose_transformer.py', name='pose_transformer')
    ld.add_action(drill_pose_transformer)

    serial_writer = Node(package='serialcomm', executable='serialwriter_exec', name='serial_writer')
    ld.add_action(serial_writer)

    return ld
