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


    return ld
