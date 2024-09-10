from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Set up the robot_state_publisher node
    moveit_cartesian_node = Node(
        package='paradocs_control',
        executable='static_obstacles',
        output='screen',
        namespace='/lbr',  # Set the namespace to 'lbr'
        remappings=[
            ("lbr/attached_collision_object", "attached_collision_object"),
            ("lbr/trajectory_execution_event", "trajectory_execution_event"),
            ("lbr/compute_cartesian_path", "compute_cartesian_path"),
            ("lbr/get_planner_params", "get_planner_params"),
            ("lbr/query_planner_interface", "query_planner_interface"),
            ("lbr/set_planner_params", "set_planner_params"),

            ("lbr/move_action/_action/get_result", "move_action/_action/get_result"),
            ("lbr/move_action/_action/send_goal", "move_action/_action/send_goal"),
            ("lbr/move_action/_action/cancel_goal", "move_action/_action/cancel_goal"),
            ("lbr/move_action/_action/status", "move_action/_action/status"),
            ("lbr/move_action/_action/feedback", "move_action/_action/feedback"),

            ("lbr/execute_trajectory/_action/get_result", "execute_trajectory/_action/get_result"),
            ("lbr/execute_trajectory/_action/send_goal", "execute_trajectory/_action/send_goal"),
            ("lbr/execute_trajectory/_action/cancel_goal", "execute_trajectory/_action/cancel_goal"),
            ("lbr/execute_trajectory/_action/status", "execute_trajectory/_action/status"),
            ("lbr/execute_trajectory/_action/feedback", "execute_trajectory/_action/feedback"),
            # ("joint_states", "lbr/joint_states"),
        ],
    )
    
    return LaunchDescription([
        moveit_cartesian_node
    ])