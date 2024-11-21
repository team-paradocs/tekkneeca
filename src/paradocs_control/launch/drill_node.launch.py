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

    serial_writer = Node(package='serialcomm', executable='serialwriter_exec', name='serial_writer')
    ld.add_action(serial_writer)

    return ld
