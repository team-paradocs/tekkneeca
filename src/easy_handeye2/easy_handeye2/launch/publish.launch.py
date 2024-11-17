from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arg_name = DeclareLaunchArgument('name')

    arg_name = DeclareLaunchArgument('name',             
            default_value=PathJoinSubstitution([
            FindPackageShare('paradocs_control'),  # Finds the install/share directory for your package
            TextSubstitution(text='config/eih_cam1')  # Appends the relative path to your file
        ]),)

    handeye_publisher = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher', parameters=[{
        'name': LaunchConfiguration('name'),
    }])

    return LaunchDescription([
        arg_name,
        handeye_publisher,
    ])
