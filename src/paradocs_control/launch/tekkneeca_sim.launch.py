from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "sim.launch.py",
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

    arg_name_d435 = DeclareLaunchArgument('nameD435',             
                default_value=PathJoinSubstitution([
                FindPackageShare('paradocs_control'),  # Finds the install/share directory for your package
                TextSubstitution(text='config/eih_cam2')  # Appends the relative path to your file
            ]),)

    handeye_publisher_d435 = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher_d435', parameters=[{
        'name': LaunchConfiguration('nameD435'),
    }])

    ld.add_action(arg_name_d435)
    ld.add_action(handeye_publisher_d435) 

    depth_optical_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_depth_optical_frame'],
    )
    ld.add_action(depth_optical_tf_publisher) 

    depth_optical_tf_publisher_d435 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'D435_link', 'camera_depth_optical_frame'],
    )
    ld.add_action(depth_optical_tf_publisher_d435) 

    drill_pose_transformer = Node(package='paradocs_control', executable='drill_pose_transformer.py', name='pose_transformer')
    ld.add_action(drill_pose_transformer)

    return ld
